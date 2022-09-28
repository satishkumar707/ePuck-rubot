from cv2 import cv
from utils.Freezeable import Freezeable
from utils import Log as MyLog
from settings import Setup
from modules.dataType import Odometry
import cv2
import numpy
import os
import math
import threading
import time

class Tracker(Freezeable, threading.Thread):
    """
    This class is used for overhead tracking.
    
    If you want to replace this tracking module, implement following methods:
    hasLocatedRobot(): Returns Boolean.
    getOdometry(): Returns current odometry of the tracked object (i.e. the robot).
    """

    def __init__(self):
        """
        Constructor. Initializes video capture
        """
        threading.Thread.__init__(self)
        
        # class name for logging messages
        self.name = "Tracker"
        # settings object
        self.setup = Setup()
        # class variables
        self.cap = None
        # tuples
        self.calibration = None
        self.track_obj = None
        self.box = None
        # Matrixes
        self.frame = None
        self.trace = None
        self.traceCvt = None
        # others
        self.odometry = Odometry()
        self.tracking_file = None
        self.calibration_file = None
        self.calibrated = False
        self.robot_located = False
        # for threading purposes
        self.stopped = False
        # see utils.Freezeable
        self.freeze()
        
        # create list for tracked markers (current blue, previous blue, curr. green, prev. green, curr. median, prev. median)
        self.track_obj = self.createList(6)
            
        # create list for converted markers (in box inertial system)
        self.box = self.createList(6)
        
        # create list for calibration markers are sorted in this way: top left, bottom left, bottom right, top right
        self.calibration = self.createList(4)
          
        # OpenCV should be up-to-date. Use version 2.4.5+
        if self.setup.other.debug:
            MyLog.d(self.name, "OpenCV should be up-to-date. Use version 2.4.5 and higher only.")
            MyLog.d(self.name, "version cv2: " + cv2.__version__)
            MyLog.d(self.name, "version cv2.cv: " + cv.__version__)
            
        # #try:    
        # CreateMat creates a matrix. CV_8UC3 says that the elements of the matrix are 8bit unsigned and tupels of length 3
        traceMat = cv.CreateMat(self.setup.image.tracking.height
                                , self.setup.image.tracking.width, cv.CV_8UC3)
        # zero out every element. Every element becomes [0,0,0]
        cv.SetZero(self.trace)
        
        traceCvtMat = cv.CreateMat(self.setup.arena.boxheight
                                   , self.setup.arena.boxwidth, cv.CV_8UC3)
        cv.SetZero(self.traceCvt)
           
        # convert traceMat and traceCvtMat to NumPy array
        self.trace = numpy.asarray(traceMat)
        self.traceCvt = numpy.asarray(traceCvtMat)  
            
        # VideoCapture, try to open the camera     
        self.cap = cv2.VideoCapture(self.setup.cam.track_id)
        
        # log success
        if self.cap.isOpened():
            MyLog.l(self.name, "Camera opened!")
            
            # grab a frame for testing purposes
            flag, self.frame = self.cap.read()
        else:
            MyLog.e(self.name, "Failed to open camera!")
            
        # check if storing the tracking data is enabled
        if self.setup.other.storepos:    
            # open and prepare .txt file to store tracking data. Create directory if needed   
            try:
                # open and prepare .txt file to store tracking data. Create directory if needed
                self.createDir(self.setup.filesystem.file_dir)
                fileName = self.setup.filesystem.file_dir + "tracking.txt"
                self.tracking_file = open(fileName, "w")
                self.tracking_file.write("x\ty\tangle\n")  # resulting in: x    y    angle
                
                MyLog.l(self.name, "File successfully written: " + fileName)
            except Exception as pokemon:
                MyLog.e(self.name, "Exception with file: " + pokemon.__str__())
            
    def run(self):
        """
        Private. Implementation for its own thread.
        Use start() instead.
        """
        MyLog.l(self.name, "Starting Tracking-Thread")
        self.asyncTracking()
        MyLog.l(self.name, "Exiting Tracking-Thread")

    def asyncTracking(self):
        """
        Private. Runs in its own thread.
        """
        while not self.stopped:
            self.updatePosition()
            
            # let thread wait 30 ms, so that it doesn't do to much work (30+ updates per second should be enough)
            time.sleep(0.03)
    
    def stop(self):
        """
        Stops tracking thread and draws trace image.
        """
        # flip converted trace image vertically
        # self.traceCvt = cv2.flip(self.traceCvt, 0)
        
        if not self.stopped:
            self.stopped = True
            
            # write unconverted trace image. If directory doesn't exist create it and write to file
            self.imwriteFile(self.setup.filesystem.file_dir + self.setup.filesystem.cam_dir
                             , "trace.png"
                             , self.trace)
            self.imwriteFile(self.setup.filesystem.file_dir + self.setup.filesystem.cam_dir
                             , "traceCvt.png"
                             , self.traceCvt)
            
            # close open files
            if self.tracking_file != None:
                try:
                    self.tracking_file.close()
                except Exception as pokemon:
                    MyLog.d(self.name, "Exception with file: " + pokemon.__str__())
            
            if self.calibration_file != None:
                self.calibration_file.close()
    
    def createList(self, size):
        """
        Create a new list with "size" points initialized
        """
        
        myTuple = []
        
        for i in range(0, size):
            myTuple.append([-1, -1])  # [-1,-1] = point with x and y coordinates
        
        return myTuple
    
    def calibrate(self):
        """
        Camera calibration (put a yellow colored object in each corner).
        Tracks the corners of the box for the robot.
        Tracked coordinates are stored counter-clockwise, beginning with top-left.
        """
        MyLog.l(self.name, "trying to calibrate with yellow markers.")
        
        # reset calibration points
        self.calibration = self.createList(4)
        
        # reset calibration points in file and prepare file. Create directory if needed
        path = self.setup.filesystem.file_dir + self.setup.filesystem.cam_dir + "calibration/"
        self.createDir(path)

        self.calibration_file = open(path + "calibration_pts.txt", "w")
        self.calibration_file.write("calibration[i][0]\t" + "calibration[i][1]\n")
        
        # #try:
        # grab a frame as NumPy array
        flag, self.frame = self.cap.read()
        
        # throw an error if  there is none
        if self.frame == None:
            raise Exception("Camera could not grab a frame.")
        
        # crop offsets of picture
        self.frame = self.frame[self.setup.image.tracking.offy:self.setup.image.tracking.offy + self.setup.image.tracking.height
                                , self.setup.image.tracking.offx:self.setup.image.tracking.offx + self.setup.image.tracking.width]
        
        quarter = None
        pos = None
        
        # defining the quarters in which to look for a yellow calibration marker
        y1 = [0
              , self.setup.image.tracking.height / 2
              , self.setup.image.tracking.height / 2
              , 0]
        y2 = [self.setup.image.tracking.height / 2
              , self.setup.image.tracking.height
              , self.setup.image.tracking.height
              , self.setup.image.tracking.height / 2]
        x1 = [0
              , 0
              , self.setup.image.tracking.width / 2
              , self.setup.image.tracking.width / 2]
        x2 = [self.setup.image.tracking.width / 2
              , self.setup.image.tracking.width / 2
              , self.setup.image.tracking.width
              , self.setup.image.tracking.width]

        # since the markers have a size bigger than one pixel we have to calculate the offsets caused by the markers.
        markerPx = self.setup.arena.markersize * self.setup.image.tracking.width / self.setup.arena.boxwidth / 2;

        # m_x and m_y determine whether to add or subtract the offset.
        m_x = [-1, -1, 1, 1];
        m_y = [-1, 1, 1, -1];

        # start manipulating frame: Make a copy of self.frame
        frameCopy = numpy.empty_like(self.frame)
        frameCopy[:] = self.frame

        # blur the image to reduce noise
        frameCopy = cv2.GaussianBlur(frameCopy
                                     , (0, 0)
                                     , 2)

        # convert to HSV channels, because a thresholding by hue catches graduated colors better than a RGB/BGR thresholding
        frameCopy = cv2.cvtColor(frameCopy
                                 , cv.CV_BGR2HSV)
            
        # #try:
        # track each marker
        
        missing_marker = False
        for i in range(0, 4):
            # crop image to the desired quarter where we are going to look for a marker
            quarter = frameCopy[y1[i]:y2[i],
                                    x1[i]:x2[i]]
  
            # get position of marker
            try:
                pos = self.getPosOfMarker(quarter
                                          , (25, 140, 110)
                                          , (35, 255, 255))
                
                if self.setup.other.debug:
                    # draw a circle on the calibration image where the marker has been found
                    cv2.circle(self.frame
                               , (int(pos[0] + x1[i])
                                  , int(pos[1] + y1[i]))
                               , 10
                               , (255, 255, 255)
                               , 2)
                
                # store position and add quarter and marker size offsets
                self.calibration[i][0] = pos[0] + x1[i] + m_x[i] * markerPx
                self.calibration[i][1] = pos[1] + y1[i] + m_y[i] * markerPx
                
                MyLog.l(self.name, "Tracked marker in quarter " + str(i) + " at " + str(pos[0]) + "," + str(pos[1]) + ". Modified position at " + str(self.calibration[i][0]) + "," + str(self.calibration[i][1]) + ".")
                    
                # save position in file
                self.calibration_file.write(str(self.calibration[i][0]) + "\t" + str(self.calibration[i][1]) + "\n")
                    
                # draw a circle on the calibration image where the marker has been found
                cv2.circle(self.frame
                           , (self.calibration[i][0]
                              , self.calibration[i][1])
                           , 20
                           , (0, 255, 255)
                           , 2)  
            except Exception, pokemon:
                MyLog.e(self.name, "Could not track marker " + str(i))  
                self.calibration_file.write("Marker " + str(i) + " not found.\n")
                missing_marker = True
                    
        if self.calibration_file != None:
            try:
                self.calibration_file.close()
            except Exception as pokemon:
                MyLog.d(self.name, "Exception with file: " + pokemon.__str__())
              
        # draw the tracked arena on the unconverted trace
        cv2.line(self.trace, (self.calibration[0][0], self.calibration[0][1]), (self.calibration[1][0], self.calibration[1][1]), (255, 0, 0), 2)
        cv2.line(self.trace, (self.calibration[1][0], self.calibration[1][1]), (self.calibration[2][0], self.calibration[2][1]), (255, 0, 0), 2)
        cv2.line(self.trace, (self.calibration[2][0], self.calibration[2][1]), (self.calibration[3][0], self.calibration[3][1]), (255, 0, 0), 2)  
        cv2.line(self.trace, (self.calibration[3][0], self.calibration[3][1]), (self.calibration[0][0], self.calibration[0][1]), (255, 0, 0), 2)   
               
        # draw calibration image
        cv2.line(self.frame, (self.calibration[0][0], self.calibration[0][1]), (self.calibration[1][0], self.calibration[1][1]), (255, 0, 0), 2)
        cv2.line(self.frame, (self.calibration[1][0], self.calibration[1][1]), (self.calibration[2][0], self.calibration[2][1]), (255, 0, 0), 2)
        cv2.line(self.frame, (self.calibration[2][0], self.calibration[2][1]), (self.calibration[3][0], self.calibration[3][1]), (255, 0, 0), 2)
        cv2.line(self.frame, (self.calibration[3][0], self.calibration[3][1]), (self.calibration[0][0], self.calibration[0][1]), (255, 0, 0), 2)
            
        # if directory doesn't exist create it and write to file
        self.imwriteFile(self.setup.filesystem.file_dir + self.setup.filesystem.cam_dir + "calibration/"
                         , "calibration.jpg", self.frame)
        
        # if a marker is missing, throw exception
        if missing_marker:
            raise Exception("There are markers missing. Restart calibration to ensure correct results.")
        else:
            # calibration finished
            self.calibrated = True
   
    def imwriteFile(self, path, fileName, _file):
        """
        Write image file.
        """
        try:
            # create directory if needed
            self.createDir(path)
            
            # write image
            cv2.imwrite(path + fileName, _file)
            MyLog.l(self.name, "File successfully written: " + fileName)
        except Exception as pokemon:
            MyLog.e(self.name, "Exception in imwriteFile: " + pokemon.__str__())
   
    def createDir(self, path):
        """
        Create directory if it doesn't exist yet.
        """
        try:
            # check if directories in path exist already
            if not os.path.exists(path):
                # create directories
                os.makedirs(path)
                MyLog.l(self.name, "Directory successfully created: " + path)
        except Exception as pokemon:
            MyLog.e(self.name, "Exception in createDir: " + pokemon.__str__())
            
    def getPosOfMarker(self, img, low, high):     
        """
        Calculate the position of an object with a given color range.
        Returns a tuple with x- and y-coordinates of tracked object.
        Only detects one object. Multiple objects (or noise in picture) will most likely lead to wrong tracking.
        """
        # position of tracked object
        pos = [0, 0]           
        x = None
        y = None
        # #try:
        # filtering the colors to be in a specific range [low, high]
        img = cv2.inRange(img, low, high)
        
        # calculate moments to estimate position of object
        mmts = cv2.moments(img)
        
        try:
            x = int(mmts["m10"] / mmts["m00"])
        except ZeroDivisionError:
            x = 0
        try:    
            y = int(mmts["m01"] / mmts["m00"])
        except ZeroDivisionError:
            y = 0
            
        # check if position is plausible: Is the detected position within our image?
        if ((x > 0) & (x < self.setup.image.tracking.width)):
            pos[0] = x
        else:
            raise Exception("x position of marker could not be tracked.")
        
        if ((y > 0) & (y < self.setup.image.tracking.height)):
            pos[1] = y
        else:
            raise Exception("y position of marker could not be tracked.")

        return pos
    
    def loadCalibration(self):
        """
        This method will be called, when you don't do a new calibration with calibrate().
        It reads calibration_pts.txt and parses it for calibration coordinates and saves them.
        Format of calibration_pts.txt:
        calibration[i][0]    calibration[i][1]
        x1    y1
        x2    y2
        x3    y3
        x4    y4
        """
        try:
            # if not already open, open calibration_file read-only
            if self.calibration_file == None:
                self.calibration_file = open(self.setup.filesystem.file_dir + self.setup.filesystem.cam_dir + "calibration/" + "calibration_pts.txt", "r")
            
            # check if file is empty
            first_character = self.calibration_file.read(1)
            if not first_character:
                raise Exception("calibration file is empty!")
            else:
                # first character wasn't empty, return to start
                self.calibration_file.seek(0)
        
            # parse each line in calibration_file, ignore first line
            i = 0
            for line in self.calibration_file:
                if not i == 0:
                    # verify that line has coordinates of a marker. If line contains "Marker", then there are no coordinates.
                    if line.find("Marker") != -1:
                        raise Exception("File is missing at least one marker. Do not load calibration. Try re-calibrating with calibrate().")
                    else:
                        # try reading two numbers from line
                        self.calibration[i - 1][0], self.calibration[i - 1][1] = line.strip().split()
                        
                        # check if read data is a number
                        if not (self.calibration[i - 1][0].isdigit() and self.calibration[i - 1][1].isdigit()):
                            raise Exception("File is missing at least one marker. Do not load calibration. Try re-calibrating with calibrate().")
                        
                        self.calibration[i - 1][0] = int(self.calibration[i - 1][0])
                        self.calibration[i - 1][1] = int(self.calibration[i - 1][1])
                i = i + 1
            
            self.calibrated = True
            MyLog.l(self.name, "Calibration successfully loaded from file!")
        except Exception as pokemon:
            MyLog.e(self.name, "Exception in loadCalibration: " + pokemon.__str__())

    def getDistance(self, p, q):
        """
        Private. Calculate distance between two given points (Phytagoras) 
        """
        return math.sqrt(((p[0] - q[0]) ** 2) + ((p[1] - q[1]) ** 2)) 
    
    def calcPosInBox(self, p, c):
        """
        Private. Calculate positions of given points within the given calibration points.
        Assumes that the box is a rectangle. Uses law of cosines. 
        """
        # first we need the distance between marker 0 to 1 and 0 to 3
        d01 = self.getDistance((c[0][0], c[0][1])
                               , (c[1][0], c[1][1]))
        d03 = self.getDistance((c[0][0], c[0][1])
                               , (c[3][0], c[3][1]))
        
        # create a list which will save the new coordinates of p
        r = self.createList(len(p))
        
        # calculate the new x- and y-position in the box with the law of cosines for each point in p
        for i in range(0, len(p)):
            # calculate x as integer
            r[i][0] = int(math.floor(((d03 ** 2) + (self.getDistance((c[0][0], c[0][1])
                                                                     , (p[i][0], p[i][1])) ** 2) 
                                      - self.getDistance((p[i][0], p[i][1])
                                                      , (c[3][0], c[3][1])) ** 2) / (2 * d03) * self.setup.arena.boxwidth / d03 + 0.5))
            # calculate y as integer
            r[i][1] = int(math.floor(((d01 ** 2) + (self.getDistance((c[0][0], c[0][1])
                                                                     , (p[i][0], p[i][1])) ** 2) 
                                      - (self.getDistance((p[i][0], p[i][1])
                                                       , (c[1][0], c[1][1])) ** 2)) / (2 * d01) * self.setup.arena.boxheight / d01 + 0.5))
        return r
    
    def drawTrace(self, trace, p, interpolated):
        """
        Private. Draw a trace given by a vector into a given image.
        """
        # draw circles and connect them with lines
        if self.setup.other.debug:
            cv2.circle(trace, (p[0][0], p[0][1]), 5, (255, 0, 0), 1)
            cv2.circle(trace, (p[2][0], p[2][1]), 5, (0, 255, 255), 1)
            cv2.line(trace, (p[1][0], p[1][1]), (p[0][0], p[0][1]), (255, 0, 0), 1)
            cv2.line(trace, (p[3][0], p[3][1]), (p[2][0], p[2][1]), (0, 255, 255), 1)
        if interpolated:
            cv2.line(trace, (int(p[5][0]), int(p[5][1])), (int(p[4][0]), int(p[4][1])), (0, 255, 255), 1)
        else:
            cv2.line(trace, (int(p[5][0]), int(p[5][1])), (int(p[4][0]), int(p[4][1])), (255, 255, 255), 1)
    
    def updatePosition(self):
        """
        Private. Updates tracked odometry.
        """
        if not self.calibrated:
            MyLog.l(self.name, "Tracker is not calibrated yet. Initializing calibration...")
            self.calibrate()
        
        angle = 0
        newPos = self.createList(2)
        
        # grab a frame as NumPy array
        flag, self.frame = self.cap.read()

        # throw an error if there is none
        if self.frame == None:
            raise Exception("Camera could not grab a frame.")

        # crop offsets of picture
        self.frame = self.frame[self.setup.image.tracking.offy:self.setup.image.tracking.offy + self.setup.image.tracking.height
                                , self.setup.image.tracking.offx:self.setup.image.tracking.offx + self.setup.image.tracking.width]

        # if the frame was not empty, we make a copy and start manipulating it
        img = numpy.empty_like(self.frame)
        img[:] = self.frame

        # blur the image to reduce noise
        img = cv2.GaussianBlur(img, (0, 0), 2)

        # convert to HSV channels, because a thresholding by hue catches graduated colors better than a RGB/BGR thresholding
        img = cv2.cvtColor(img, cv.CV_BGR2HSV)

        # get current position of blue marker
        # DEPRECATED: newPos[0] = self.getPosOfMarker(img, (85, 150, 100), (100, 255, 255))
        try:
            newPos[0] = self.getPosOfMarker(img
                                            , (90, 100, 90)
                                            , (110, 255, 255))
        except Exception, pokemon:
            newPos[0] = [-1, -1]
            
        # get current position of green marker
        # DEPRECATED: newPos[1] = self.getPosOfMarker(img, (45, 150, 100), (55, 255, 255))
        try:
            newPos[1] = self.getPosOfMarker(img
                                            , (40, 140, 90)
                                            , (55, 255, 255))
        except:
            newPos[1] = [-1, -1]
            
        # initialize tracking error flag
        trackingError = False
        
        # check if a marker could not be tracked
        if newPos[0][0] < 0 or newPos[0][1] < 0 or newPos[1][0] < 0 or newPos[1][1] < 0:
            # find out which marker could not be tracked
            marker = None
            
            if newPos[0][0] < 0 or newPos[0][1] < 0:
                marker = "blue"
            else:
                marker = "green"
                
            MyLog.e(self.name, "Could not track " + marker + " marker.")
            
            # draw circles on current marker positions and output a picture 
            cv2.circle(self.frame, (newPos[0][0], newPos[0][1]), 10, (255, 0, 0), 2)
            cv2.circle(self.frame, (newPos[1][0], newPos[1][1]), 10, (0, 255, 0), 2)
            self.imwriteFile(self.setup.filesystem.file_dir + self.setup.filesystem.cam_dir, "markerNotTracked.jpg", self.frame)
            
            # set error flag
            trackingError = True
            
        # check if distance between the two markers is too high (if both markers could be tracked)
        if not trackingError and self.getDistance((newPos[0][0], newPos[0][1]), (newPos[1][0], newPos[1][1])) > self.setup.arena.markerdist:
            MyLog.e(self.name, "Robot markers too distant. There was probably a mistake trying to detect a marker.")
            
            # set error flag
            trackingError = True
            
            # draw circles on current marker positions and output a picture 
            cv2.circle(self.frame, (newPos[0][0], newPos[0][1]), 10, (255, 0, 0), 2)
            cv2.circle(self.frame, (newPos[1][0], newPos[1][1]), 10, (0, 255, 0), 2)
            self.imwriteFile(self.setup.filesystem.file_dir + self.setup.filesystem.cam_dir + "calibration/"
                             , "tooDistant.jpg", self.frame)
            
            
        # handle tracking error
        if trackingError:
            # if we have trace data, try linear interpolation to guess the current position
            if self.track_obj[1][0] >= 0 or self.track_obj[3][0] >= 0:
                newPos[0][0] = self.track_obj[0][0] + (self.track_obj[0][0] - self.track_obj[1][0])
                newPos[0][1] = self.track_obj[0][1] + (self.track_obj[0][1] - self.track_obj[1][1])
                newPos[1][0] = self.track_obj[2][0] + (self.track_obj[2][0] - self.track_obj[3][0])
                newPos[1][1] = self.track_obj[2][1] + (self.track_obj[2][1] - self.track_obj[3][1])
                
                MyLog.l(self.name, "Interpolated position 1: " + str(newPos[0][0]) + "," + str(newPos[0][1]))
                MyLog.l(self.name, "Interpolated position 2: " + str(newPos[1][0]) + "," + str(newPos[1][1]))
            # else, restore the last known position
            else:
                newPos[0] = self.track_obj[0]
                newPos[1] = self.track_obj[2]
                MyLog.l(self.name, "Cannot interpolate. Restored last position.")
        
        # save the last position, adopt the new position 
        self.track_obj[1][0] = self.track_obj[0][0]
        self.track_obj[1][1] = self.track_obj[0][1]
        self.track_obj[0][0] = newPos[0][0]
        self.track_obj[0][1] = newPos[0][1]
        self.track_obj[3][0] = self.track_obj[2][0]
        self.track_obj[3][1] = self.track_obj[2][1]
        self.track_obj[2][0] = newPos[1][0]
        self.track_obj[2][1] = newPos[1][1]
        self.track_obj[5][0] = self.track_obj[4][0]
        self.track_obj[5][1] = self.track_obj[4][1]
        
        # build the median of marker points
        self.track_obj[4][0] = (self.track_obj[0][0] + self.track_obj[2][0]) / 2
        self.track_obj[4][1] = (self.track_obj[0][1] + self.track_obj[2][1]) / 2
        
        # calculate the robots' position within the inertial system of the box
        self.box = self.calcPosInBox(self.track_obj, self.calibration)
        
        # update the robots' odometry
        self.odometry.location[0] = self.box[4][0]
        self.odometry.location[1] = self.box[4][1]
        
        # next, calculate the robots' angle. Determine distance between blue and green marker
        dist = self.getDistance((self.box[0][0], self.box[0][1]), (self.box[2][0], self.box[2][1]))

        if  dist != 0:
            # angle = asin((blue.y - green.y)/distance)
            angle = math.asin((self.box[0][1] - self.box[2][1]) / dist) * self.setup.constants.RAD2DEG

            # since asin is defined only between -90 and 90 degrees, we need to differentiate to gain 91-180 degrees, too
            if self.box[0][0] < self.box[2][0]:
                if angle > 0:
                    angle = 180 - angle
                else:
                    angle = -180 - angle
                
            self.odometry.angle = math.floor(angle + 0.5)
        
        # draw trace from input data and from aligned data
        self.drawTrace(self.trace
                       , self.track_obj
                       , trackingError)
        self.drawTrace(self.traceCvt
                       , self.box
                       , trackingError)
        
        # write current position and angle in log-file
        try:
            if self.setup.other.storepos:
                self.tracking_file.write(str(self.box[4][0]) + "\t" + str(self.box[4][1]) + "\t" + str(self.odometry.angle) + "\n")
        except Exception, pokemon:
            MyLog.e(self.name, pokemon)
        
    def getOdometry(self):
        return self.odometry
    
    def locateRobot(self):
        """
        Detect robots' current position.
        """
        if not self.calibrated:
            MyLog.l(self.name, "Tracker is not calibrated yet. Initializing calibration...")
            self.calibrate()
        
        newPos = self.createList(2)
        
        # grab a frame as NumPy array
        flag, self.frame = self.cap.read()
        
        # throw an error if there is none
        if self.frame == None:
            raise Exception("Camera could not grab a frame.")
        
        # crop offsets of picture
        self.frame = self.frame[self.setup.image.tracking.offy:self.setup.image.tracking.offy + self.setup.image.tracking.height
                                , self.setup.image.tracking.offx:self.setup.image.tracking.offx + self.setup.image.tracking.width]
        
        # if the frame was not empty, we make a copy and start manipulating it
        img = numpy.empty_like(self.frame)
        img[:] = self.frame

        # blur the image to reduce noise
        img = cv2.GaussianBlur(img, (0, 0), 2)

        # convert to HSV channels, because a thresholding by hue catches graduated colors better than a RGB/BGR thresholding
        img = cv2.cvtColor(img, cv.CV_BGR2HSV)
    
        # get current position of blue marker
        newPos[0] = self.getPosOfMarker(img, (90, 100, 90), (110, 255, 255))
        
        # get current position of green marker
        newPos[1] = self.getPosOfMarker(img, (40, 140, 90), (55, 255, 255))
        
        # initialize tracking error flag and 
        trackingError = False
        
        # check if a marker could not be tracked
        if newPos[0][0] < 0 or newPos[0][1] < 0 or newPos[1][0] < 0 or newPos[1][1] < 0:
            # find out which marker could not be tracked
            marker = None
            
            if newPos[0][0] < 0 or newPos[0][1] < 0:
                marker = "blue"
            else:
                marker = "green"
                
            MyLog.e(self.name, "Could not track " + marker + " marker.")
            
            # draw circles on current marker positions and output a picture 
            cv2.circle(self.frame, (newPos[0][0], newPos[0][1]), 10, (255, 0, 0), 2)
            cv2.circle(self.frame, (newPos[1][0], newPos[1][1]), 10, (0, 255, 0), 2)
            self.imwriteFile(self.setup.filesystem.file_dir + self.setup.filesystem.cam_dir, "markerNotTracked.jpg", self.frame)
            
            # set error flag
            trackingError = True
            
        # check if distance between the two markers is too high (if both markers could be tracked)
        if not trackingError and self.getDistance((newPos[0][0], newPos[0][1]), (newPos[1][0], newPos[1][1])) > self.setup.arena.markerdist:
            MyLog.e(self.name, "Robot markers too distant. There was probably a mistake trying to detect a marker.")
            
            # set error flag
            trackingError = True
            
            # draw circles on current marker positions and output a picture 
            cv2.circle(self.frame, (newPos[0][0], newPos[0][1]), 10, (255, 0, 0), 2)
            cv2.circle(self.frame, (newPos[1][0], newPos[1][1]), 10, (0, 255, 0), 2)
            self.imwriteFile(self.setup.filesystem.file_dir + self.setup.filesystem.cam_dir + "calibration/", "tooDistant.jpg", self.frame)
            
        # handle tracking error
        if trackingError:
            raise Exception("Could not track robot.")
        
        # save the last position, adopt the new position 
        self.track_obj[1][0] = self.track_obj[0][0]
        self.track_obj[1][1] = self.track_obj[0][1]
        self.track_obj[0][0] = newPos[0][0]
        self.track_obj[0][1] = newPos[0][1]
        self.track_obj[3][0] = self.track_obj[2][0]
        self.track_obj[3][1] = self.track_obj[2][1]
        self.track_obj[2][0] = newPos[1][0]
        self.track_obj[2][1] = newPos[1][1]
        self.track_obj[5][0] = self.track_obj[4][0]
        self.track_obj[5][1] = self.track_obj[4][1]
        
        # build the median of marker points
        self.track_obj[4][0] = (self.track_obj[0][0] + self.track_obj[2][0]) / 2
        self.track_obj[4][1] = (self.track_obj[0][1] + self.track_obj[2][1]) / 2
        
        # calculate the robots' position within the inertial system of the box
        self.box = self.calcPosInBox(self.track_obj, self.calibration)
        
        # update the robots' odometry
        self.odometry.location[0] = self.box[4][0]
        self.odometry.location[1] = self.box[4][1]
        
        # next, calculate the robots' angle. Determine distance between blue and green marker
        dist = self.getDistance((self.box[0][0], self.box[0][1]), (self.box[2][0], self.box[2][1]))

        if  dist != 0:
            # angle = asin((blue.y - green.y)/distance)
            self.odometry.angle = math.asin((self.box[0][1] - self.box[2][1]) / dist) * self.setup.constants.RAD2DEG
            
            # since asin is defined only between -90 and 90 degrees, we need to differentiate to gain 91-180 degrees, too
            if self.box[0][0] < self.box[2][0]:
                if self.odometry.angle > 0:
                    self.odometry.angle = 180 - self.odometry.angle
                else:
                    self.odometry.angle = -180 - self.odometry.angle
        
        MyLog.l(self.name, "Robots' position: " + str(self.odometry.location) + " angle: " + str(self.odometry.angle) + " degree")
        
        # start position of robot is correct now
        self.robot_located = True

    def hasLocatedRobot(self):
        """
        Returns true if the robots' correct start position is known.
        """
        return self.robot_located
