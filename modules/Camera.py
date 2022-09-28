from utils.Freezeable import Freezeable
from utils import Log as MyLog
from settings import Setup
import cv2
from cv2 import cv
import os
from threading import Thread
from time import sleep
from time import time

class Camera(Freezeable):
    """
    This class implements a thread that can collect and save pictures from
    a camera (i.e. to collect data for SFA-training purposes).
    
    public methods:
    start()
    stop()
    setFileName(String)
    """

    def __init__(self):
        """
        Constructor
        """
        # Setup variables
        self.setup = Setup()
        
        self.name = "Robots' Camera"
        
        # file name of pictures that will be saved (i.e. "filename_000001.png")
        self.file_name = "frame"
        
        # user specified file extension (png/jpg or other OpenCV supported extensions)
        self.file_extension = ".png"
        
        # flag to decide wether to use a sharpening filter on grabbed pictures or not
        self.sharpening = False
        
        # picture height and width (format which will be cut out from original frame)
        self.width_range = self.setup.image.robot.width_range
        self.height_range = self.setup.image.robot.height_range
        
        # picture height and width (format to which will be scaled)
        self.pic_height = self.setup.image.robot.pic_height
        self.pic_width = self.setup.image.robot.pic_width
        
        # cut out picture from original frame before rescaling it
        self.do_cut = True
        
        # scale pictures before saving them
        self.do_scale = True
        
        # counting images
        self.count = 0
        
        # for collectFrames(number); Collect up to "number" frames
        self.collect_up_to = 0 
        
        # class' thread
        self._thread = None
        
        # monitors the status of this class' thread
        self.active = False
        
        # camera variable
        self.cap = None
        
        # file path
        self.path = self.setup.filesystem.file_dir + self.setup.filesystem.cam_dir + "data/"
        
        # observer list for classes that want to process images from this camera in real time
        self._observers = []
        
        self.freeze()
        
        # VideoCapture, try to open the camera      
        self.cap = cv2.VideoCapture(self.setup.cam.robot_id)
        
        if self.cap == None:
            raise Exception("There was an unexpected error trying to create an object of cv2.VideoCapture(" + str(self.setup.cam.robot_id) + ")!")
        
        # log success
        if self.cap.isOpened():
            MyLog.l(self.name, "Camera opened!")
            
            # grab a frame for testing purposes
            flag, self.frame = self.cap.read()
        else:
            raise Exception("Camera could not be opened!")
        
        try:
            # check if directories in path exist already
            if not os.path.exists(self.path):
                # create directories
                os.makedirs(self.path)
                MyLog.l(self.name, "Directory successfully created: " + self.path)
        except Exception as pokemon:
            MyLog.e(self.name, "Exception creating directories: " + pokemon.__str__())
    
    def start(self):
        """
        Start a thread that collects pictures.
        """
        try:
            if not self.active:
                MyLog.l(self.name, "Starting thread...")
                self.active = True
                # hacky solution, so that you can stop() collectFrames(), too.
                # like this start() will grab and save at maximum 999999 pictures, but this number will
                # not be reached anyways
                self.collect_up_to = 999999
                self._thread = Thread(target=self.cameraThread, args=[])
                self._thread.start()
            else:
                MyLog.l(self.name, "Thread is running already. start() did nothing.")
        except Exception, pokemon:
            MyLog.e(self.name, "Exception in thread: " + pokemon.__str__())
    
    def stop(self):
        """
        Stop thread.
        """
        if self.active:
            MyLog.l(self.name, "Stopping thread...")
            self.active = False
        else:    
            MyLog.l(self.name, "Thread is not active. stop() did nothing.")
        
    def cameraThread(self):
        """
        Private. This method is running in its own thread.
        """
        # variable needed for collectFrames(x). Example: Starting count can be set to 500. 
        # Then collectFrames(5) has to loop until count is 500 + 5.
        collect_up_to = self.count + self.collect_up_to
        current_time = time()
        # loop while active (start() and stop()) or until x frames have been collected (when collectFrames(x) was called)
        while self.active and self.count < collect_up_to:
            self.grabImage()
            sleep(0.03)
        self.active = False
        
        time_passed = time() - current_time
        MyLog.l(self.name, "Thread stopped. Time passed: %ss. Frames collected: %d " % (time_passed, self.count - self.setup.runparams.cam_start_count))
    
    def grabImage(self):
        """
        Private. This method grabs one frame, filters and resizes it and saves it.
        """
        # grab a frame as NumPy array
        flag, frame = self.cap.read()
        
        # throw an error if there is none
        if frame == None:
            raise Exception("Camera could not grab a frame.")
        
        # if filter-flag is set
        if self.sharpening:
            # blur the image to reduce noise
            image = cv2.GaussianBlur(frame, (0, 0), 2)
            frame = cv2.addWeighted(frame, 2, image, -1, 0)
        
        # resize picture
        if self.do_scale:
            frame = cv2.resize(frame, (self.pic_width, self.pic_height), 0, 0, cv.CV_INTER_AREA)
        
                # cut out picture in right dimensions (550*350)
        if self.do_cut:
            frame = frame[self.height_range[0]:self.height_range[1], self.width_range[0]:self.width_range[1]]    
        
        # build file path, pad number with zeroes from left side
        file_path = self.path + self.file_name + "_" + str(self.count).zfill(6) + self.file_extension
        
        # increase counter
        self.count = self.count + 1
        
        if not self.setup.runparams.enable_SFA:
            try:
                # save image
                cv2.imwrite(file_path, frame)
            except Exception, pokemon:
                MyLog.e(self.name, "Exception in grabImage(): Could not write file:" + pokemon.__str__())
        
        # send new picture to observers (e.g.PlaceCellCalculation), so that they can process it in real time
        self.notify(frame)
    
    def collectFrames(self, number):
        """
        Collect up to "number" frames.
        """
        if not self.active:
            self.collect_up_to = number
            self.active = True
            MyLog.l(self.name, "Starting thread...")
            self._thread = Thread(target=self.cameraThread, args=[])
            self._thread.start()
        else:
            MyLog.l(self.name, "Thread is running already. collectFrames() did nothing.")
    
    def setCount(self, number):
        """
        Set the counting variable for saving pictures to "number".
        """
        self.count = number
    
    def setFileName(self, name):
        """
        Set file name of pictures that will be saved (i.e. "filename_000001.png").
        """    
        self.file_name = name
    
    def setPicCut(self, width_range, height_range):
        """
        Define the area which will be cut out (before rescaling) from original frame.
        Example for a 550*350 (default) picture:
        setPicCut((83, 633), (100, 450))
        """
        self.width_range = width_range
        self.height_range = height_range
    
    def setPicScale(self, width, height):
        """
        Define to which format pictures will be scaled (has no effect when setPicResize(False) was called).
        """
        self.pic_width = width
        self.pic_height = height
       
    def setDoCut(self, _bool):
        """
        Default: True
        Set flag to cut out pictures.
        """
        self.do_cut = _bool 
    
    def setDoScale(self, _bool):
        """
        Default: True
        Set flag to resize pictures.
        """
        self.do_scale = _bool   
    
    def initCam(self):
        """
        Call this method >>once<< before you start recording to setup the camera. 
        After that call the settings won't change until you restart the computer.
        """
        # setup camera. Example: v4lctl -c /dev/video0 setnorm PAL
        os.system("v4lctl -c /dev/video" + str(self.setup.cam.robot_id) + " setnorm " + self.setup.cam.norm)
        os.system("v4lctl -c /dev/video" + str(self.setup.cam.robot_id) + " setinput " + self.setup.cam.input)
        
        MyLog.l(self.name, "Robots' cam initialized.")
        
    def waitForCompletion(self):
        """
        Wait in calling thread until thread finishes (i.e. wait until x frames were collected when collectFrames(x) was called).
        """
        try:
            if self._thread != None:
                self._thread.join()
        except Exception, pokemon:
            MyLog.e(self.name, "Exception in waitForCompletion(): " + pokemon.__str__())
            
    # ## OBSERVER PATTERN
    def attach(self, observer):
        if not observer in self._observers:
            self._observers.append(observer)
            MyLog.l(self.name, observer.name + " is now observing " + self.name)
            
    def detach(self, observer):
        try:
            self._observers.remove(observer)
        except:
            pass
        
    def notify(self, pic):
        """
        Notifiy observers that there was a change => There is a new picture available.
        """
        for observer in self._observers:
            observer.update(pic)
    
