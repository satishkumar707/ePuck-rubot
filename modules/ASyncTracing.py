from utils.Freezeable import Freezeable
from cv2 import cv
from settings import Setup
from utils import Log as MyLog
import cv2
import numpy
import threading
import os
import time

class ASyncTracing(Freezeable, threading.Thread):
    """
    This class draws the robots' trace from his view.
    Start tracking by calling start(). Stop by calling stop().
    You can start this class at most once. To continue drawing the trace
    after you have stopped, create a new ASyncTracing object and start() it.
    If you want to reset the trace file call clearImage().
    """
    def __init__(self, ePuckControl):
        """
        Constructor
        """
        threading.Thread.__init__(self)
        
        # class variables
        self.setup = Setup()
        self.myName = "ASyncTracing"
        
        # monitors the status of this class' thread
        self.active = False
        
        # An object of this class can be used at most once.
        # You can call getUsed() to check if it was used already.
        self.used = False
        
        self.trace = None
        self._file = None
        self.epuck = ePuckControl
        
        self.image_path = self.setup.filesystem.file_dir + self.setup.filesystem.epuck_dir
        self.image_file_name = "trace.png"
        self.txt_file_name = "tracing.txt"
        
        self.freeze()
        
        # open trace image
        try:
            self.trace = cv2.imread(self.image_path + self.image_file_name)
        except:
            MyLog.e(self.myName, "Exception in __init__() when trying to read image file: " + self.image_path + self.image_file_name)

        if self.trace == None:
            self.prepareFilesystem()
    
    def prepareFilesystem(self):
        """
        Creates needed directories and files for tracing.
        """
        # if trace didn't exist, create it
        try:
            # create directory if needed
            if not os.path.exists(self.image_path):
                # create directories
                os.makedirs(self.image_path)
                MyLog.l(self.myName, "Directory successfully created: " + self.image_path)
                
            # CreateMat creates a matrix. CV_8UC3 says that the elements of the matrix are 8bit unsigned and tupels of length 3
            traceMat = cv.CreateMat(self.setup.arena.boxheight, self.setup.arena.boxwidth, cv.CV_8UC3)
            # zero out every element. Every element becomes [0,0,0]
            cv.SetZero(traceMat)
            
            # convert traceMat as NumPy array
            self.trace = numpy.asarray(traceMat)
        except Exception as pokemon:
            MyLog.e(self.myName, "Exception in prepareFilesystem: " + pokemon.__str__())
                
        # if option active, prepare .txt-file for writing
        if self.setup.other.storepos:
            try:
                # create directory if needed
                path = self.setup.filesystem.file_dir + self.setup.filesystem.epuck_dir
                if not os.path.exists(path):
                    # create directories
                    os.makedirs(path)
                    MyLog.l(self.myName, "Directory successfully created: " + path)
            
                # open file
                self._file = open(path + self.txt_file_name, "a")
                MyLog.l(self.myName, "File successfully opened: " + path + self.txt_file_name)
            except Exception as pokemon:
                MyLog.e(self.myName, "Exception in prepareFilesystem: " + pokemon.__str__())

    def run(self):
        """
        Private. Implementation for its own thread.
        Use start() instead.
        """
        if not self.used:
            MyLog.l(self.myName, "Starting Tracing-Thread")
            self.active = True
            self.used = True
            self.asyncTracing()
            MyLog.l(self.myName, "Exiting Tracing-Thread")
        
        # object was used before
        else:
            # log warning
            MyLog.d(self.myName, "This object was used before, run() did nothing. Create a new object for further tracing.")
        
    def clearImage(self):
        """
        Deletes trace image and txt file and prepares new files.
        """
        try:
            # remove image and .txt file
            os.remove(self.image_path + self.image_file_name)
        except:
            pass
        try:
            os.remove(self.image_path + self.txt_file_name)
        except:
            pass
            
        self.trace = None
        
        try:
            self.prepareFilesystem()
        except Exception, pokemon:
            MyLog.e(self.name, "Exception in clearImage(): " + pokemon.__str__())

    def stop(self):
        """
        Stops tracking thread and draws trace image.
        """
        # write trace to image file
        try:
            # create directory if needed
            if not os.path.exists(self.image_path):
                # create directories
                os.makedirs(self.image_path)
                MyLog.l(self.myName, "Directory successfully created: " + self.image_path)
            
            # write image
            cv2.imwrite(self.image_path + self.image_file_name, self.trace)
            MyLog.l(self.myName, "File successfully written: " + self.image_path + self.image_file_name)
        except Exception as pokemon:
            MyLog.e(self.myName, "Exception in run: " + pokemon.__str__())
                
        self.active = False
        
    def asyncTracing(self):
        while self.active:
            # get old and current position of ePuck
            old = self.epuck.getOldOdometry().location
            current = self.epuck.getOdometry().location
            
            # check if the robots' position was corrected by another module
            isCorrection = self.epuck.getCorrectionStatus()
            
            # draw line between old and current position
            if not isCorrection[0]:
                cv2.line(self.trace, (int(old[0]), int(old[1])), (int(current[0]), int(current[1])), (0, 255, 0), 1)
            elif not isCorrection[1]:
                # draw correction in red
                cv2.line(self.trace, (int(old[0]), int(old[1])), (int(current[0]), int(current[1])), (0, 0, 255), 1)
                self.epuck.setCorrectionStatus((False, False))
            else:
                self.epuck.setCorrectionStatus((False, False))
                if self.setup.other.debug:
                    # this is the start-position correction. Draw it white when debug is on
                    cv2.line(self.trace, (int(old[0]), int(old[1])), (int(current[0]), int(current[1])), (255, 255, 255), 1)
                
            # write to file in every call when in debug mode
            if self.setup.other.debug:
                cv2.imwrite(self.image_path + self.image_file_name, self.trace)
            
            # if option active, also store data in a .txt-file
            if self.setup.other.storepos:
                try:
                    self._file.write(str(current[0]) + "\t" + str(current[1]) + "\t" + str(self.epuck.getOdometry().angle) + "\n")
                except Exception, pokemon:
                    MyLog.e(self.myName, pokemon)

            # wait a moment (33ms)
            time.sleep(0.033)
        
    def isUsed(self):
        """
        Returns true if this object was started already.
        """
        return self.used
