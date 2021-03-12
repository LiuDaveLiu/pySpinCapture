# Jason Keller
# Feb 2020
# =============================================================================
#  Program to set BlackFly S camera settings and acquire frames from 2 synchronized cameras and 
#  write them to a compressed video file. Based on FLIR Spinnaker API example code. I have 
#  also tested with a Flea3 camera, which works but requires modifying the camera
#  settings section to use non "Quickspin" functions (see FLIR examples). 
# 
#  The intent is that this program started first, then will wait for triggers
#  on Line 0 (OPTO_IN) from the DAQ system. It is assumed that the DAQ system will provide
#  a specified number of triggers, and that the Line 0 black wires of both cameras are
#  soldered together and driven simultaneously. Both cameras output their "exposure active"
#  signal on Line 1 (OPTO_OUT, the white wire, which is pulled up to 3.3V via a 1.8kOhm resistor 
#  for each camera) so that each frame can be synchronized (DAQ should sample this at ~1kHz+).
#
#  Tkinter is used to provide a simple GUI to display the images, and skvideo 
#  is used as a wrapper to ffmpeg to write H.264 compressed video quickly, using
#  mostly default parameters (although I tried pix_fmt gray to reduce size further,
#  but default worked better)
#
#  To setup, you must download an FFMPEG executable and set an environment 
#  variable path to it (as well as setFFmpegPath function below). Other nonstandard
#  dependencies are the FLIR Spinnaker camera driver and PySpin package (see 
#  Spinnaker downloads), and the skvideo package. 
#  
#  NOTE: currently there is no check to see if readout can keep up with triggering
#  other that a timeout warning. It is up to the user to determine if the correct number
#  of frames are captured. Also, the "ffmpegThreads" parameter can throttle CPU usage
#  by FFMPEG to allow other data acquistion task priority. For example, with an Intel Xeon
#  W-2145 processor and 4 threads, CPU usage is limited to ~50-60% @ 500Hz, 320x240px,
#  and compressed writing is close to real-time.
#
# TO DO:
# (1) report potential # missed frames (maybe use counter to count Line 1 edges and write to video file)
# (2) try using ImageEvent instead of blocking GetNextImage(timeout) call
# (3) explicitly setup camera onboard buffer
# (4) use multiprocess or other package to implement better parallel processing
# (5) try FFMPEG GPU acceleration: https://developer.nvidia.com/ffmpeg
# =============================================================================
import PySpin, time, os, threading, queue
from datetime import datetime
import tkinter as tk
from PIL import Image, ImageTk
import numpy as np
import skvideo
skvideo.setFFmpegPath('C:/ffmpeg/bin') #set path to ffmpeg installation before importing io
import skvideo.io

#constants
SAVE_FOLDER_ROOT = 'D:/video/bottom'
FILENAME_ROOT = 'mj_' # optional identifier
EXPOSURE_TIME = 2000 #in microseconds
WAIT_TIME = 0.001 #in seconds - this limits polling time and should be less than the frame rate period 
GAIN_VALUE1 = 25 #in dB, 0-40;
GAMMA_VALUE = 0.5 #0.25-1

IMAGE_HEIGHT = 540  #540 pixels default
IMAGE_WIDTH = 720 #720 pixels default
HEIGHT_OFFSET1 = 0 #round((540-IMAGE_HEIGHT)/2) # Y, to keep in middle of sensor
WIDTH_OFFSET1 = 0 #round((720-IMAGE_WIDTH)/2) # X, to keep in middle of sensor

FRAMES_TO_RECORD = 10000 #frame rate * num seconds to record; this should match # expected exposure triggers from DAQ counter output
TRIALS_TO_RECORD = 1000 #Max number of trials
CAM_TIMEOUT = 100 #in ms; time to wait for another image before aborting

# generate output video directory and filename and make sure not overwriting
now = datetime.now()
mouseStr = input("Enter mouse ID: ")
saveFolder = SAVE_FOLDER_ROOT + '/' + mouseStr

if not os.path.exists(saveFolder):
    os.mkdir(saveFolder)
    
dateStr = now.strftime("%Y_%m_%d") #save folder ex: 2020_01_01
timeStr = now.strftime("%H_%M_%S") #filename ex: mj_09_30_59.mp4
saveFolder = SAVE_FOLDER_ROOT + '/' + mouseStr + '/' + dateStr

if not os.path.exists(saveFolder):
    os.mkdir(saveFolder)

os.chdir(saveFolder)

j = 0 #trials
fullFilePath = saveFolder + '/'
print('Video will be saved to: {}'.format(fullFilePath))

# get frame rate and query for video length based on this
print('# frames = {:d}'.format(FRAMES_TO_RECORD))

# SETUP FUNCTIONS #############################################################################################################
def initCam1(cam): #function to initialize camera parameters for synchronized capture

    cam.Init()
    # load default configuration
    cam.UserSetSelector.SetValue(PySpin.UserSetSelector_Default)
    cam.UserSetLoad()

    # set acquisition. Continues acquisition. Auto exposure off. Set frame rate using exposure time. 
    cam.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)
    cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Off)
    cam.ExposureMode.SetValue(PySpin.ExposureMode_Timed) #Timed or TriggerWidth (must comment out trigger parameters other that Line)
    cam.ExposureTime.SetValue(EXPOSURE_TIME)
    cam.AcquisitionFrameRateEnable.SetValue(False)

    # set analog. Set Gain + Gamma. 
    cam.GainAuto.SetValue(PySpin.GainAuto_Off)
    cam.Gain.SetValue(GAIN_VALUE1)
    cam.GammaEnable.SetValue(True)
    cam.Gamma.SetValue(GAMMA_VALUE)

    # set ADC bit depth and image pixel depth, size
    cam.AdcBitDepth.SetValue(PySpin.AdcBitDepth_Bit8)
    cam.PixelFormat.SetValue(PySpin.PixelFormat_Mono8)
    cam.Width.SetValue(IMAGE_WIDTH)
    cam.Height.SetValue(IMAGE_HEIGHT)
    cam.OffsetX.SetValue(WIDTH_OFFSET1)
    cam.OffsetY.SetValue(HEIGHT_OFFSET1)

    # setup FIFO buffer
    camTransferLayerStream = cam.GetTLStreamNodeMap()
    handling_mode1 = PySpin.CEnumerationPtr(camTransferLayerStream.GetNode('StreamBufferHandlingMode'))
    handling_mode_entry = handling_mode1.GetEntryByName('OldestFirst')
    handling_mode1.SetIntValue(handling_mode_entry.GetValue())

    # set trigger input to Line0 (the black wire)
    cam.TriggerMode.SetValue(PySpin.TriggerMode_On)
    cam.TriggerOverlap.SetValue(PySpin.TriggerOverlap_ReadOut) #Off or ReadOut to speed up
    cam.TriggerSource.SetValue(PySpin.TriggerSource_Line0)
    cam.TriggerActivation.SetValue(PySpin.TriggerActivation_RisingEdge) #LevelHigh or RisingEdge
    cam.TriggerSelector.SetValue(PySpin.TriggerSelector_FrameStart) # require trigger for each frame

    # optionally send exposure active signal on Line 2 (the white wire)
    cam.LineSelector.SetValue(PySpin.LineSelector_Line1)
    cam.LineMode.SetValue(PySpin.LineMode_Output) 
    cam.LineSource.SetValue(PySpin.LineSource_ExposureActive) #route desired output to Line 1 (try Counter0Active or ExposureActive)
  
def saveImage(imageWriteQueue, writer): #function to save video frames from the queue in a separate process
    while True:
        dequeuedImage = imageWriteQueue.get()
        if dequeuedImage is None:
            break
        else:
            writer.writeFrame(dequeuedImage)
            imageWriteQueue.task_done()

def camCapture(camQueue, cam, k): #function to capture images, convert to numpy, send to queue, and release from buffer in separate process
    global tEnd
    while True:
        if k == 0: #wait infinitely for trigger for first image
            image = cam.GetNextImage() #get pointer to next image in camera buffer; blocks until image arrives via USB, within infinite timeout for first frame while waiting for DAQ to start sending triggers    
        else:
            try:
                image = cam.GetNextImage(CAM_TIMEOUT) #get pointer to next image in camera buffer; blocks until image arrives via USB, within CAM_TIMEOUT
            except: #PySpin will throw an exception upon timeout, so end gracefully
                print(str(k) + ' frames captured')
                tEnd = 1
                
                break
        npImage = np.array(image.GetData(), dtype="uint8").reshape( (image.GetHeight(), image.GetWidth()) ); #convert PySpin ImagePtr into numpy array
        camQueue.put(npImage)  
        image.Release() #release from camera buffer
        k = k + 1
    

# INITIALIZE CAMERAS & COMPRESSION ###########################################################################################
system = PySpin.System.GetInstance() # Get camera system
cam_list = system.GetCameras() # Get camera list
cam1 = cam_list[0]
initCam1(cam1)  

# setup output video file parameters (can try H265 in future for better compression):  
# for some reason FFMPEG takes exponentially longer to write at nonstandard frame rates, so just use default 25fps and change elsewhere if needed
crfOut = 23 #controls tradeoff between quality and storage, see https://trac.ffmpeg.org/wiki/Encode/H.264 
ffmpegThreads = 20 #this controls tradeoff between CPU usage and memory usage; video writes can take a long time if this value is low
#crfOut = 18 #this should look nearly lossless

#setup tkinter GUI (non-blocking, i.e. without mainloop) to output images to screen quickly
window = tk.Tk()
window.title("camera acquisition")
geomStrWidth = str(IMAGE_WIDTH + 25)
geomStrHeight = str(IMAGE_HEIGHT + 35)
window.geometry(geomStrWidth + 'x' + geomStrHeight) # 2x width+25 x height+35; large enough for frames from 2 cameras + text
textlbl = tk.Label(window, text="waiting for trigger...")
textlbl.grid(column=0, row=0)
imglabel = tk.Label(window) # make Label widget to hold image
imglabel.place(x=10, y=20) #pixels from top-left
window.update() #update TCL tasks to make window appear

#############################################################################
# start main program loop ###################################################
#############################################################################    

try:
    print('Press Ctrl-C to exit early and save video')
    
    for j in range(TRIALS_TO_RECORD):
        i = 0 #frames
        movieName1 = mouseStr + '_' + dateStr + '_bottom_' + str(j) + '.mp4'
        #writer1 = skvideo.io.FFmpegWriter(movieName1, outputdict={'-vcodec': 'libx264', '-crf': str(crfOut), '-threads': str(ffmpegThreads)})
        writer1 = skvideo.io.FFmpegWriter(movieName1, outputdict={'-vcodec': 'h264_nvenc', '-preset': 'p7', '-level': '6.2'})
        imageWriteQueue1 = queue.Queue() #queue to pass images captures to separate compress and save thread
        cam1Queue = queue.Queue()  #queue to pass images from separate cam1 acquisition thread
        # setup separate threads to accelerate image acquisition and saving, and start immediately:
        save1Thread = threading.Thread(target=saveImage, args=(imageWriteQueue1, writer1,))
        cam1Thread = threading.Thread(target=camCapture, args=(cam1Queue, cam1, i,))
        save1Thread.start()  
        cam1.BeginAcquisition()
        cam1Thread.start()
        tEnd = 0

        for i in range(FRAMES_TO_RECORD): # main acquisition loop
                                
            while cam1Queue.empty(): #wait until ready in a loop
                time.sleep(WAIT_TIME)
                if tEnd == 1:
                    break
            
            if tEnd == 1:
                cam1.EndAcquisition()              
                imageWriteQueue1.join() #wait until compression and saving queue is done writing to disk
                writer1.close() #close to FFMPEG writer
                break
                
            if i == 0:
                tStart = time.time()
                print('Capture begins')
               
            dequeuedAcq1 = cam1Queue.get() # get images formated as numpy from separate process queues as soon as they are both ready

            #imageWriteQueue.put(enqueuedImageCombined) #put next combined image in saving queue
            imageWriteQueue1.put(dequeuedAcq1)

            if (i+1)%20 == 0: #update screen every X frames
            #if (i+1): #update screen every X frames            
                framesElapsedStr = "frame #: " + str(i+1) + " of " + str(FRAMES_TO_RECORD)
                textlbl.configure(text=framesElapsedStr)
                I = ImageTk.PhotoImage(Image.fromarray(dequeuedAcq1))
                imglabel.configure(image=I)
                imglabel.image = I #keep reference to image
                window.update() #update on screen (this must be called from main thread)

            if (i+1) == (FRAMES_TO_RECORD):
                print('Complete ' + str(i+1) + ' frames captured')
                tEndAcq = time.time()
        
# end aqcuisition loop #############################################################################################            

except KeyboardInterrupt: #if user hits Ctrl-C, everything should end gracefully
    tEndAcq = time.time()
    pass        

cam1.EndAcquisition()
tEndWrite = time.time()
print('File written at: {:.2f}sec'.format(tEndWrite - tStart))
window.destroy() 

# delete all pointers/variable/etc:
cam1.DeInit()
del cam1
cam_list.Clear()
del cam_list
system.ReleaseInstance()
del system
print('Done!')