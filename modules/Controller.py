from modules.Camera import Camera
from modules.Navigation import ePuckControl
from modules.PlaceCellCalculation import PlaceCellCalculation
from modules.Tracker import Tracker
from settings import Setup
from time import sleep
from utils.Freezeable import Freezeable
from utils import Log as MyLog

class Controller(Freezeable):
    """
    This class is used to manage all modules for an experiment, including
    an ePuck, a tracking-module and a camera-module.
    """
    def __init__(self):
        # load config
        self.setup = Setup()
        self.use_tracking = self.setup.runparams.use_tracking      # for tracking set tracking in settings.py to 1
        self.use_cam      = self.setup.runparams.use_cam           # for epuck cam set use_cam in settings.py to 1
        self.new_calib    = self.setup.runparams.new_calibration   # to calibrate set new_calibration in settings.py to 1
        self.use_SFA      = self.setup.runparams.enable_SFA        # to use SFA network, set enable_SFA to 1
        # prepare variables
        self.tracker = Tracker() if self.use_tracking else None
        self.cam     = Camera()  if self.use_cam and not self.new_calib else None
        # freeze
        self.freeze()
    
    def checkSettings(self):
        """
        This method raises an exception and stops execution 
        flow if the runparameters in settings.py are incorrect.
        """
        if self.setup.runparams.enable_SFA and not self.setup.runparams.use_cam:
            raise Exception("wrong input: SFA enabled, cam disabled: Set use_cam to 1.")
        
        if self.setup.runparams.enable_SFA and not self.setup.runparams.use_tracking:
            raise Exception("wrong input: SFA enabled, tracking disabled: Set use_tracking to 1.")
        
        if self.setup.runparams.new_calibration and not self.setup.runparams.use_tracking:
            raise Exception("wrong input: calibration enabled, tracking disabled: Set use_tracking to 1.")
        
        if self.setup.runparams.init_cam and not self.setup.runparams.use_cam:
            raise Exception("wrong input: init_cam enabled, cam disabled: Set use_cam to 1.")
        
        if self.setup.runparams.limit_cond == "frames" and not self.setup.runparams.use_cam:
            raise Exception("wrong input: You started an experiment with limit_cond = \"frames\", but disabled cam. Set use_cam to 1!")

        if self.setup.runparams.limit_cond == "frames" and self.setup.runparams.limit_val <= 0:
            raise Exception("wrong input: You started an experiment with limit_cond = \"frames\", but limit_val <= 0! Set limit_val to something bigger than 0 (number of frames you want to collect).")                            
    
    def startExperiment(self):
        """
        Start robot and its modules.
        """
        self.checkSettings()
        MyLog.createLogFile("/local/ePuck/", "log.txt")
        
        if self.setup.runparams.new_calibration:
            # if tracking module is available, update start position
            if self.tracker != None:
                # you can call this function in every execution, but you don't need to
                # if the position of your camera and markers does not change. 
                # calibrate() will save your calibration in files and read them afterwards. Thus, call it once.
                self.tracker.calibrate()
            else:
                raise Exception("wrong input: You need to activate the tracker-module for calibration!")
        # load calibration from file (if tracking-module is available) and start experiment
        elif self.setup.runparams.init_cam:
            self.cam.initCam()
        else:
            self.pcCalc = PlaceCellCalculation(self.tracker) if self.use_SFA and self.cam != None else None
            self.navigation = ePuckControl(self.setup.robot.mac, self.tracker, self.pcCalc) if not self.new_calib else None
            if self.tracker != None:
                # load calibration from .txt-file
                self.tracker.loadCalibration()
        
                # track robots' start position so that ctrl gets initialized with the correct position
                if self.setup.runparams.correction_mode == 1:
                    self.tracker.locateRobot()
            
                    # if tracker was able to locate the robots' position, then update robots' start position
                    if self.tracker.hasLocatedRobot():
                        self.navigation.adjustStartPositionViaTracker()
                
                # start tracking-module
                self.tracker.start()

            if self.cam != None:
                # configure cam-module
                self.cam.setCount(self.setup.runparams.cam_start_count)
                self.cam.setDoCut(self.setup.image.robot.do_cut)
                self.cam.setDoScale(self.setup.image.robot.do_scale)
                
                if self.setup.runparams.enable_SFA:
                    # register pcCalculation class at camera
                    self.cam.attach(self.pcCalc)
                    
            if self.setup.runparams.limit_cond == "path":
                # prepare follow-path experiment
                if self.cam != None:
                    self.cam.start()
                self.navigation.followPath(self.setup.filesystem.epuck_follow_path)
                self.navigation.waitForCompletion()
                if self.cam != None:
                    self.cam.stop()                
            
            elif self.setup.runparams.limit_cond == "frames":
                if self.cam != None:
                    # let cam-module collect x frames (if input was correct)
                    if self.setup.runparams.limit_val > 0:
                        self.cam.collectFrames(self.setup.runparams.limit_val)
           
            elif self.setup.runparams.limit_cond == "time":
                # start cam-module
                if self.cam != None:
                    self.cam.start()
            else:
                raise Exception("wrong input: limit_cond. Correct usage: limit_cond = (\"frames\", \"time\", \"path\")")
            
            if not self.setup.runparams.limit_cond == "path":
                # start ePuck in followPath- or randomWalk-mode
                if self.setup.runparams.epuck_nav_param == -1:
                    self.navigation.followPath(self.setup.filesystem.epuck_follow_path)
                elif self.setup.runparams.epuck_nav_param >= 0 and self.setup.runparams.epuck_nav_param <= 1:
                    self.navigation.startRandomWalk(self.setup.runparams.epuck_nav_param)
                else:
                    raise Exception("wrong input: epuck_nav_param has to be in [0, 1] or -1!")
            
                if self.setup.runparams.limit_cond == "frames":
                    # stop cam and robot if cam has finished collecting x frames
                    self.cam.waitForCompletion()
                    self.navigation.stop()
                else:
                    # stop cam and robot after x seconds
                    sleep(self.setup.runparams.limit_val)
                    self.navigation.stop()
                    if self.cam != None:
                        self.cam.stop()
        
            if self.tracker != None:
                # disable tracking-module
                self.tracker.stop()
            
            if self.setup.runparams.enable_SFA:
                self.pcCalc.saveActivityData()
