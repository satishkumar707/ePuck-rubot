from modules.dataType import Odometry
from settings import Setup
from utils import Log as MyLog
from utils.Freezeable import Freezeable
import cPickle as pickle
import numpy as np

class PlaceCellCalculation(Freezeable):
    """
    This class is part of an observer pattern. 
    It has to be registered at another class,
    which then sends pictures to this class.
    These images are sent to a trained SFA (slow feature analysis)
    network and the answer of this network will be processed.
    The result will be a four dimensional vector. The first two components
    are x- and y-coordinates, the third component is the averaged answer of 
    the SFA network and the fourth component is a direction. This vector will be
    saved to file at the end of an experiment.
    
    FUTURE WORK: Calculation of robots' odometry with SFA (slow feature analysis)
    FUTURE WORK: This class can be used as a "tracking module".
    """
    def __init__(self, tracker):
        """
        Constructor.
        """
        # setup variables
        self.setup = Setup()
        
        self.name = "PlaceCellCalculation"
        
        # list containing place cell activity images as black grey scale canvas
        self.spikeImage = []
        
        self._thread = None
        
        # monitor thread status (running/stopped)
        self.active = False
        
        # tracking-module, so that the robots' position is known at all times
        self.tracker = tracker
        
        # if the robots' start position was calculated, this variable is True (FUTURE WORK)
        self.robot_located = False
        
        # robots' odometry calculated by SFA-stuff (FUTURE WORK)
        self.odometry = Odometry()
        
        # open and load SFA network
        MyLog.l(self.name, "Loading SFA network...")
        tsn_file = open(self.setup.filesystem.network_file
                            , "r")
        self.sfa_network = pickle.load(tsn_file)
        MyLog.l(self.name, "SFA network successfully loaded!")
        
        # data array for network answers
        self.data = np.zeros((self.setup.arena.boxwidth
                              , self.setup.arena.boxheight
                              , 32
                              , 2))
        
        # data array to check if self.data got a network answer at a particular position already.
        # It is called "activity" because this array can tell if there was SFA-activity.  
        # This is needed for calculations in self.data
        # 0 means there was no activity yet, 1 means there was activitiy at position (x,y) in direction (left/right)
        self.activity = np.zeros((self.setup.arena.boxwidth
                              , self.setup.arena.boxheight + 20
                              , 2))
        
        self.freeze()
        
    def getOdometry(self):
        """
        TO BE IMPlEMENTED.
        """
        return self.odometry
    
    def getNetworkAnswer(self, pic):
        """
        Send a picture to the SFA network and return its answer.
        """
        img_height = pic.shape[0]
        img_width = pic.shape[1]
    
        # insert data into array
        data = np.array(pic, np.float32).reshape(1, img_width * img_height * 3)
        
        return self.sfa_network.execute(data)
    
    def update(self, pic):
        """
        As soon as the observed class notifies its observers 
        (=> there is a new picture available), this function
        will be called.
        It asks the SFA network for the picture related answer, 
        checks the robots' direction and saves the SFA answer 
        into a data array.
        """
        
        if self.tracker != None:
            tracking_answer = self.tracker.getOdometry()
        else:
            raise Exception("No tracking-module initialized. Impossible to use SFA.")
        try:
            # if robot is at a valid position
            if tracking_answer.isValidLocation():
                # send picture to SFA network and get answer
                sfa_answer = self.getNetworkAnswer(pic)
                
                tracking_loc = tracking_answer.location
                # current direction of robot
                _dir = tracking_answer.angle
                
                # angle which should be considered
                angle = 60
                
                if self.setup.runparams.segment_orientation == "up_down":
                    if _dir > (90 - angle/2) and _dir < (90 + angle/2):
                        # _dir = 1 means robot is looking up
                        _dir = 1
                    elif _dir < (-90 + angle/2) or _dir > (-90 - angle/2):
                        # _dir = 0 means robot is looking down
                        _dir = 0
                    else:
                        # robots' direction is not valid
                        _dir = None
                elif self.setup.runparams.segment_orientation == "left_right":
                    if _dir > -(angle/2) and _dir < (angle/2):
                        # _dir = 1 means robot is looking right
                        _dir = 1
                    elif _dir < (-180 + angle/2) or _dir > (180 - angle/2):
                        # _dir = 0 means robot is looking left
                        _dir = 0
                    else:
                        # robots' direction is not valid
                        _dir = None
                
                # if robots' direction is valid (i.e. he is clearly moving left/right)
                if _dir != None:
                    # store network answer in data array
                    # if there was no activity at that position yet (activity == 0), just replace data with network answer
                    if self.activity[int(tracking_loc[0])
                                     , int(tracking_loc[1])
                                     , _dir] == 0:  
                        for i in range(0, 32):
                            self.data[int(tracking_loc[0])
                                      , int(tracking_loc[1])
                                      , i
                                      , _dir] = sfa_answer[0][i]
                        
                        self.activity[int(tracking_loc[0])
                                      , int(tracking_loc[1])
                                      , _dir] = 1
                    # there was activity already, so calculate the average
                    else:
                        for i in range(0, 32):
                            self.data[int(tracking_loc[0])
                                      , int(tracking_loc[1])
                                      , i
                                      , _dir] += sfa_answer[0][i]
                            self.data[int(tracking_loc[0])
                                      , int(tracking_loc[1])
                                      , i
                                      , _dir] /= 2
        except Exception, pokemon:
            MyLog.e(self.name, "Exception in update(): " + pokemon.__str__())

    def saveActivityData(self):
        try:
            # save raw data to file
            file_path = self.setup.filesystem.file_dir + self.setup.filesystem.activity_path
            pickle.dump(self.data, open(file_path + "cell_data.dat", "wb"), 1)
            MyLog.l(self.name, "Activity data saved: " + file_path + "cell_data.dat")
        except:
            MyLog.e(self.name, "Error saving activity data. Path: " + file_path + "cell_data.dat")
