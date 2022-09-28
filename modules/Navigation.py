import math

from threading  import Thread
from libs.ePuck import ePuck
from numpy import sin, sign, cos, degrees, arcsin, radians
from utils.Freezeable import Freezeable
from settings import Setup
from modules.dataType import Odometry
from utils import Log as MyLog
from ASyncTracing import ASyncTracing
from time import sleep
from modules.randomVehicle import randomVehicle
 
class ePuckControl(ePuck, Freezeable):
    """
    This class is used to control the movement of an ePuck.
    Examples:
    -goTo(2000,2000)
    -followPath("./path.txt")
    -waitForCompletion()
    -startRandomWalk(0.5)
    """
    def __init__(self, mac, tracker, sfa_calc):
        # initialize ePuck
        ePuck.__init__(self, mac)
        
        # constants
        self.POS_MAX = 2 ** 15 - 1  # wheel_limit
        self.FULL_TURN = 1278
        self.TICKS_PER_M = 7700
        self.MOTOR_HACK = self.POS_MAX / 2  # hack for motor encoders. Motor encoders go from -2**15 to 2**15-1
        self.ENCODER_HACK = 1000
        
        self.path_length = 0      # setup variables
        self.dir = 0              # current direction
        self.d_enc_l = 0          # difference in old and new motor encoder left
        self.d_enc_r = 0          # difference in old and new motor encoder right
        
        # motor encoder hack to fix overflows
        self.motor_pos_old = [self.MOTOR_HACK, self.MOTOR_HACK]
        
        self.name = "ePuckControl"
        self.setup = Setup()
        
        # new and old location, angle
        self.odometry = Odometry()
        self.odometry.location = [self.setup.robot.diameter,
                                  self.setup.robot.diameter]
        self.old_odometry = Odometry()
        self.old_odometry.location = [self.setup.robot.diameter,
                                      self.setup.robot.diameter]
        
        self.motor_pos = None                  # motor position left and right
        self._thread   = None                  # variable to start and stop a thread
        self.target    = [0, 0]                # current target from goTo command
        self.tracer    = ASyncTracing(self)    # module: robots' own tracing
        self.tracker = tracker                 # module: (camera-)tracking
        self.sfa_calc = sfa_calc               # module: SFA-network calculator
        
        # variable for tracking-correction. "Update as soon as the position error surpasses a threshold."
        self.error_threshold = self.setup.robot.diameter * self.setup.robot.error_threshold

        # self.is_corrected[0] is true, if the robots position was corrected
        # self.is_corrected[1] is true, if the robots start-position was corrected
        self.is_corrected = [False, False]
        
        # module: random walk
        self.random_vehicle = None
        
        # what is the robot doing right now?
        self.random_walking    = False
        self.is_following_path = False
        self.is_turning        = False
        
        # variable to stop the robot manually
        self.stopped = False
        
        # reset trace-image to be empty
        self.tracer.clearImage()
        
        self.freeze()

        # connect to ePuck
        self.tryConnecting()
        
        # enable motor sensors
        self.enable("motor_position")
        self.enable("motor_speed")
        
        # hack for motor encoders
        try:
            self.set_motor_position(self.MOTOR_HACK, self.MOTOR_HACK)
            self.step()
        except Exception, pokemon:
            MyLog.e(self.name, "Exception in __init__: " + pokemon.__str__())
      
    def tryConnecting(self):
        """
        Try connecting to ePuck
        """
        MyLog.l(self.name, "Connecting to ePuck...")
        connected = False
        
        # try "tries"-times to connect to ePuck, if it fails raise exception
        tries   = 5
        counter = 0
        while (not connected) and counter < tries:
            try:
                self.connect()
                connected = True
            except Exception:
                MyLog.d(self.name, "Connecting to ePuck: Attempt " + str(counter + 1) + " failed...")
                counter = counter + 1
        
        if not connected:
            raise Exception("Could not connect to ePuck. Turn it on or try again.")
        
        MyLog.l(self.name, "Connection to ePuck successfully established.")
                
    def checkEncoders(self):
        """
        Check for overflow in encoders and adjust them.
        """
        _return = True
        # left wheel is going forwards, adjust if encoder is going to overflow
        # MODULO-CLASS
        if self.motor_pos_old[0] > self.POS_MAX - self.ENCODER_HACK and sign(self.d_enc_l) == 1:
            self.motor_pos_old[0] = (self.POS_MAX - self.motor_pos_old[0]) % (self.POS_MAX - self.ENCODER_HACK)
            self.motor_pos_old[1] = self.motor_pos_old[1] % (self.POS_MAX - self.ENCODER_HACK)
            self.set_motor_position(self.motor_pos_old[0], self.motor_pos_old[1])
            self.step()
            _return = False
        # right wheel is going forwards, adjust if encoder is going to overflow
        # MODULO-CLASS
        if self.motor_pos_old[1] > self.POS_MAX - self.ENCODER_HACK and sign(self.d_enc_r) == 1:
            self.motor_pos_old[1] = (self.POS_MAX - self.motor_pos_old[1]) % (self.POS_MAX - self.ENCODER_HACK)
            self.motor_pos_old[0] = self.motor_pos_old[0] % (self.POS_MAX - self.ENCODER_HACK)
            self.set_motor_position(self.motor_pos_old[0], self.motor_pos_old[1])
            self.step()
            _return = False
        # left wheel is going backwards, adjust if encoder is going to overflow
        # MODULO-CLASS
        if self.motor_pos_old[0] < self.ENCODER_HACK and sign(self.d_enc_l) == -1:
            self.motor_pos_old[0] = (self.POS_MAX - self.motor_pos_old[0]) % (self.POS_MAX - self.ENCODER_HACK)
            self.motor_pos_old[1] = self.motor_pos_old[1] % (self.POS_MAX - self.ENCODER_HACK)
            self.set_motor_position(self.motor_pos_old[0], self.motor_pos_old[1])
            self.step()
            _return = False
        # right wheel is going backwards, adjust if encoder is going to overflow
        # MODULO-CLASS
        if self.motor_pos_old[1] < self.ENCODER_HACK and sign(self.d_enc_r) == -1:
            self.motor_pos_old[1] = (self.POS_MAX - self.motor_pos_old[1]) % (self.POS_MAX - self.ENCODER_HACK)
            self.motor_pos_old[0] = self.motor_pos_old[0] % (self.POS_MAX - self.ENCODER_HACK)
            self.set_motor_position(self.motor_pos_old[0], self.motor_pos_old[1])
            self.step()
            _return = False
        return _return

    def updatePosition(self):
        """
        Update the robots' odometry and encoders.
        """
        try: 
            # update robots' actuators and sensors
            self.step()
                
            # get current value of motor encoders
            self.motor_pos = self.get_motor_position()
        except Exception, pokemon:
            MyLog.e(self.name, "Exception in updatePosition: " + pokemon.__str__())
                    
        # calculate change of motor encoders left and right      
        self.d_enc_l = self.motor_pos[0] - self.motor_pos_old[0]           
        self.d_enc_r = self.motor_pos[1] - self.motor_pos_old[1]

        # and remember motor position for next step
        self.motor_pos_old[0] = self.motor_pos[0]
        self.motor_pos_old[1] = self.motor_pos[1]

        # calculate change of distance
        d_path_length = (self.d_enc_r + self.d_enc_l) * 0.5 * 1000 / self.TICKS_PER_M  

        # calculate change of angle 
        d_dir = (self.d_enc_l - self.d_enc_r) * 0.5 * 360 / self.FULL_TURN

        # update accumulated path length       
        self.path_length += d_path_length 

        # calculate change in x- and y-direction
        d_x = math.floor(d_path_length * cos(radians(self.odometry.angle + d_dir * 0.5)) + 0.5)     
        d_y = math.floor(d_path_length * sin(radians(self.odometry.angle + d_dir * 0.5)) + 0.5)
        
        # remember old odometry for drawing purpose
        self.old_odometry.location[0] = self.odometry.location[0]
        self.old_odometry.location[1] = self.odometry.location[1]
        self.old_odometry.angle = self.odometry.angle

        # update current odometry
        self.odometry.location[0] += d_x
        self.odometry.location[1] += d_y
        self.odometry.angle += d_dir

        # if |angle| is > 180, add/subtract 360deg to keep angles within one peroid
        if self.odometry.angle > 180:
            self.odometry.angle -= 360
        if self.odometry.angle < -180:
            self.odometry.angle += 360

        # adjust robot with help of a tracking module (if correction was enabled in settings.py)
        if self.tracker != None and self.setup.runparams.correction_mode == 1:
        #if self.tracker != None and self.setup.runparams.correction_mode == 1 and not self.is_turning:
            self.adjustViaTracker()

        # adjust robot with help of a SFA module (if correction was enabled in settings.py)
        if self.sfa_calc != None and self.setup.runparams.correction_mode == 2 and not self.is_turning:
            self.adjustViaSFA()   

        # update sensors and actuators
        try:
            self.checkEncoders()
        except Exception, pokemon:
            MyLog.e(self.name, "Exception in updatePosition: " + pokemon.__str__())

    def adjustStartPositionViaTracker(self):
        """
        Adjust robots odometry with help of a tracking module. 
        Used at start.
        """
        trackOdo = self.tracker.getOdometry()
        self.odometry.location[0] = trackOdo.location[0]
        self.odometry.location[1] = trackOdo.location[1]
        self.odometry.angle = trackOdo.angle

        self.old_odometry.location[0] = trackOdo.location[0]
        self.old_odometry.location[1] = trackOdo.location[1]
        self.old_odometry.angle = trackOdo.angle
        
        self.is_corrected = [True, True]

    def adjustViaSFA(self):
        """
        TO BE IMPLEMENTED (FUTURE WORK).
        Use self.sfa_calc to calculate robots' odometry
        and then update this class' odometry. 
        An example to implement that update is written below.   
        """
        # this example acts on the assumption that sfa-calc already calculated the robots' odometry
        SFAOdo = self.sfa_calc.getOdometry()
        self.odometry.location[0] = SFAOdo.location[0]
        self.odometry.location[1] = SFAOdo.location[1]
        self.odometry.angle = SFAOdo.angle
        self.is_corrected = [True, True]

    def adjustViaTracker(self):
        """
        Adjust robots odometry once in a while with help of a tracking module.
        """
        track_odometry = self.tracker.getOdometry()

        # calculate the difference of the robots' position and the real position the tracking module tracked 
        difference = self.calcDistance(track_odometry.location, self.odometry.location)
        angle_diff = self.calcAngleDiff(track_odometry.angle, self.odometry.angle)

        # only adjust if position error surpasses self.error_threshold
        if difference >= self.error_threshold or angle_diff >= self.setup.robot.angle_err_threshold:
            # replace own odometry with odometry from tracking module
            self.odometry.location[0] = track_odometry.location[0]
            self.odometry.location[1] = track_odometry.location[1]
            self.odometry.angle = track_odometry.angle
            
            self.is_corrected[0] = True

    def turn(self, degree):
        """
        Turn robot by "degree" degree.
        """
        self.is_turning = True

        slow = False

        # remember old speed
        old_speed = None
        while old_speed == None:
            try:
                self.updatePosition()
                old_speed = self.get_motor_speed()
            except Exception, pokemon:
                MyLog.e(self.name, "Exception in turn(): " + pokemon.__str__())

        # determine direction-variable
        dir_w = None
        if sign(degree) == 1:
            dir_w = 0
        else:
            dir_w = 1

        # determine motor-encoder value at target angle
        new_pos = (self.motor_pos[dir_w] + abs(degree) * self.FULL_TURN / 360) % (self.POS_MAX - self.ENCODER_HACK)
        
        if (new_pos - self.motor_pos[dir_w] >= 50) and slow == False:
            turn_speed = [0,0]
            while (turn_speed[0] != 400 or turn_speed[1] != 2**16-400) and (turn_speed[0] != 2**16-400 or turn_speed[1] != 400):
                try:
                    if dir_w == 0:
                        self.set_motors_speed(400, -400)
                    else:
                        self.set_motors_speed(-400, 400) 
                    self.updatePosition()
                    turn_speed = self.get_motor_speed()
                except Exception, pokemon:
                    MyLog.e(self.name, "Exception in turn(): " + pokemon.__str__())

        # determine motor-encoder value at target angle again, for safety reasons
        # MODULO-CLASS
        new_pos = (self.motor_pos[dir_w] + abs(degree) * self.FULL_TURN / 360) % (self.POS_MAX - self.ENCODER_HACK)
        
        # there is a known issue, that if the robot turns and the encoder
        # is resetted, the robot turns until the encoder is at its maximum again
        # example: new_pos = 30000, but encoders will be resetted at 30005:
        # encoder is 30010 and gets resetted to 1010. So it is still smaller than 30000,
        # so the encoders grows until it maybe hits 30002 and does not get resetted immediately.
        # This hack should fix this problem by putting new_pos a bit further, meaning the robot will
        # turn further, but avoid that bug.
        TURN_HACK = 75
        if new_pos <= TURN_HACK:
            new_pos += TURN_HACK
            
        if new_pos >= (self.POS_MAX - self.ENCODER_HACK - TURN_HACK):
            new_pos -= TURN_HACK
        
        # while not at target angle and the robot was not stopped: turn
        # MODULO-CLASS (see TURN_HACK)
        while (self.motor_pos[dir_w] <= new_pos) and not self.stopped:
            try:
                self.updatePosition()
            except Exception, pokemon:
                MyLog.e(self.name, "Exception in turn(): " + pokemon.__str__())

            if (new_pos - self.motor_pos[dir_w] < 50) and slow == False:
                try:
                    self.set_motors_speed(sign(degree) * 150, sign(degree) * -150)
                    slow = True
                except Exception, pokemon:
                    MyLog.e(self.name, "Exception in turn(): " + pokemon.__str__())
            try:
                self.updatePosition()
            except Exception, pokemon:
                MyLog.e(self.name, "Exception in turn(): " + pokemon.__str__())
            
        self.is_turning = False
        
    def goTo(self, targetp):
        """
        Go to coordinates in targetp.
        """
        try:
            MyLog.l(self.name, "Starting thread: goTo" + str(targetp[0]) + "," + str(targetp[1]))
            self._thread = Thread(target=self.threadedGoTo, args=[targetp])
            self._thread.start()
        except Exception, pokemon:
            MyLog.e(self.name, "Exception in goTo: " + pokemon.__str__())
            
    def threadedGoTo(self, pos):
        """
        Let robot go to point "pos". 
        Is called in its own thread.
        """
        d_s = self.calcDistance(pos, self.odometry.location)
        if d_s > self.error_threshold:
            # current position was calculated by the robot, not by the tracking module
            self.is_corrected[0] = False
        
            self.path_length = 0
            
            # start robots' tracing
            if not self.is_following_path:
                if self.tracer.isUsed():
                    # tracer was used before, so create a new one to continue tracing
                    self.tracer = ASyncTracing(self)
                self.tracer.start()

            end = self.path_length + d_s
            alpha = degrees(arcsin((pos[1] - self.odometry.location[1]) / d_s))
            if alpha >= 0 and self.odometry.location[0] > pos[0]:
                alpha = 180 - alpha
            elif alpha < 0 and self.odometry.location[0] > pos[0]:
                alpha = -180 - alpha

            phi_turn = (alpha - self.odometry.angle) % 360

            # keep turning angle in [-180, 180]
            if phi_turn > 180:
                phi_turn = phi_turn - 360
            elif phi_turn < -180:
                phi_turn = phi_turn + 360

            # only turn if the turn angle is bigger than |epsilon|
            epsilon = 2
            if phi_turn < -epsilon or phi_turn > epsilon:
                self.turn((phi_turn))
            
            finished = False
            while not finished and not self.is_corrected[0]:
                try:
                    self.set_motors_speed(700, 700)
                    self.step()
                    finished = True
                except Exception, pokemon:
                    MyLog.e(self.name, "Exception1 in threadedGoTo: " + pokemon.__str__())

            # while not at target position and the robot was not stopped
            while self.path_length < end and not self.is_corrected[0] and not self.stopped:
                try:
                    self.updatePosition()
                except Exception, pokemon:
                    MyLog.e(self.name, "Exception2 in threadedGoTo, going straight: " + pokemon.__str__())
                    
            # if robot wasn't corrected by another module, it will end the goTo-command here
            if not self.is_corrected[0]:
                try:
                    self.zeroWheelspeed()
                except Exception, pokemon:
                    MyLog.e(self.name, "Exception3 in threadedGoTo, stopping: " + pokemon.__str__())
            
                # ePuck reached target
                if not self.stopped:
                    MyLog.l(self.name, "ePuck reached target (" + str(pos[0]) + "," + str(pos[1]) + ").")
                
                # stop robots' tracing
                if not self.is_following_path:
                    self.tracer.stop()
            # robot was corrected, so repeat goTo-command
            else:
                # robot is correcting path
                try:
                    self.updatePosition()
                except Exception, pokemon:
                    MyLog.e(self.name, "Exception in threadedGoTo, correcting path: " + pokemon.__str__())
                self.threadedGoTo(pos)
        else:
            MyLog.d(self.name, "goTo(" + str(pos[0]) + "," + str(pos[1]) + "): ePuck is already at target. Skipping goTo.")        

    def zeroWheelspeed(self):
        """
        Fail-safe method to set the robots' wheelspeed to (0,0) and therefore stop it.
        """
        motor_speed = None
        try:
            
            motor_speed = self.get_motor_speed()
        except Exception, pokemon:
            MyLog.e(self.name, "Exception in zeroWheelspeed(): Couldn't get_motor_speed(). Trying again. " + pokemon.__str__())
        
        # loop as long as the robot did not stop and try to stop him.
        while motor_speed == None or motor_speed[0] != 0 or motor_speed[1] != 0:
            try:
                self.set_motors_speed(0, 0)
                self.updatePosition()
                sleep(0.033)
                motor_speed = self.get_motor_speed()
            except Exception, pokemon:
                MyLog.e(self.name, "Exception in zeroWheelspeed(): Couldn't stop robot. " + pokemon.__str__())

    def followPath(self, filePath):
        """
        Follow path in .txt-file. Format of file:
        x    y
        4000    2000
        500    500
        1234    4321
        ...
        """
        try:
            MyLog.l(self.name, "Starting thread: followPath: " + filePath)
            self.is_following_path = True
            self._thread = Thread(target=self.threadedFollowPath, args=[filePath])
            self._thread.start()
        except Exception, pokemon:
            MyLog.e(self.name, "Exception in followPath: " + pokemon.__str__())
            
    def threadedFollowPath(self, filePath):
        """
        Let robot follow a path read in a .txt-file. 
        Is called in its own thread.
        """
        path = "".join(filePath)

        try:
            _file = None
            # check if filePath is existing and not empty
            if path != None or not path == "":
                _file = open(path, "r")
            
            # check if file is empty
            first_character = _file.read(1)
            if not first_character:
                raise Exception("filePath is empty!")
            else:
                # first character wasn't empty, return to start
                _file.seek(0)
        
            # prepare variable for coordinates
            p = []
            
            # parse each line in _file to check if file is correct and save read coordinates into variable p. Ignore first line
            i = 0
            for line in _file:
                if not i == 0:
                    # append new point
                    p.append([-1, -1])
                    
                    # try reading two numbers from line
                    p[i - 1][0], p[i - 1][1] = line.strip().split()
                        
                    # check if read data is a number
                    if not (p[i - 1][0].isdigit() and p[i - 1][1].isdigit()):
                        raise Exception("File is corrupt. Read data wasn't int.")
                    
                    # make sure p contains numbers
                    p[i - 1][0] = int(p[i - 1][0])
                    p[i - 1][1] = int(p[i - 1][1])
                i = i + 1
                
            _file.close()
            
            # start tracing robot
            if self.tracer.isUsed():
                # tracer was used before, so create a new one to continue tracing
                self.tracer = ASyncTracing(self)
            self.tracer.start()      
            
            # iterate through points and tell the robot to go to each point
            for point in p:
                # if robot was not stopped manually continue following path
                if not self.stopped:
                    self.threadedGoTo(point)
            
            # stop tracing robot
            self.tracer.stop()
            
            self.is_following_path = False
            MyLog.l(self.name, "finished following path.")
        except Exception as pokemon:
            MyLog.e(self.name, "Exception in threadedFollowPath: " + pokemon.__str__())
            
    def loopPath(self, filePath, turns):
        """
        Follow path in .txt-file. If end is reached, repeat 
        from first point ("turns"-times). Format of file:
        x    y
        4000    2000
        500    500
        1234    4321
        ...
        """
        try:
            MyLog.l(self.name, "Starting thread: loopPath: " + filePath)
            self.is_following_path = True
            self._thread = Thread(target=self.threadedLoopPath, args=[filePath, turns])
            self._thread.start()
        except Exception, pokemon:
            MyLog.e(self.name, "Exception in loopPath: " + pokemon.__str__())
            
    def threadedLoopPath(self, filePath, turns):
        """
        Let robot follow a path read in a .txt-file. Is called in its own thread.
        """
        path = "".join(filePath)

        try:
            _file = None
            # check if filePath is existing and not empty
            if path != None or not path == "":
                _file = open(path, "r")
            
            # check if file is empty
            first_character = _file.read(1)
            if not first_character:
                raise Exception("filePath is empty!")
            else:
                # first character wasn't empty, return to start
                _file.seek(0)
        
            # prepare variable for coordinates
            p = []
            
            # parse each line in _file to check if file is correct and save read coordinates into variable p. Ignore first line
            i = 0
            for line in _file:
                if not i == 0:
                    # append new point
                    p.append([-1, -1])
                    
                    # try reading two numbers from line
                    p[i - 1][0], p[i - 1][1] = line.strip().split()
                        
                    # check if read data is a number
                    if not (p[i - 1][0].isdigit() and p[i - 1][1].isdigit()):
                        raise Exception("File is corrupt. Read data wasn't int.")
                    
                    # make sure p contains numbers
                    p[i - 1][0] = int(p[i - 1][0])
                    p[i - 1][1] = int(p[i - 1][1])
                i = i + 1
                
            _file.close()
            
            # start tracing robot
            if self.tracer.isUsed():
                # tracer was used before, so create a new one to continue tracing
                self.tracer = ASyncTracing(self)
            self.tracer.start()      
            
            # iterate through points and tell the robot to go to each point
            for i in range(0, turns):
                # if robot was not stopped manually continue following path
                if not self.stopped:
                    for point in p:
                        if not self.stopped:
                            self.threadedGoTo(point)
            
            # stop tracing robot
            self.tracer.stop()
            
            self.is_following_path = False
            MyLog.l(self.name, "finished following path.")
        except Exception as pokemon:
            MyLog.e(self.name, "Exception in threadedFollowPath: " + pokemon.__str__())
      
    def startRandomWalk(self, momentum):
        """
        Creates an object of randomVehicle, starts the random walk and traces it with ASyncTracing.
        """
        self.random_vehicle = randomVehicle(self)
        
        # start robots' tracing
        if self.tracer.isUsed():
            # tracer was used before, so create a new one to continue tracing
            self.tracer = ASyncTracing(self)
        self.tracer.start()
        
        self.random_walking = True
        self.random_vehicle.start(momentum)
            
    def stopRandomWalk(self):
        """
        Stops random walk and tracing.
        """
        self.random_vehicle.stop()
        
        self.random_walking = False
        
        # if tracking-module is available, stop it
        if self.tracker != None:
            self.tracker.stop()
      
    def stop(self):
        """
        Stop whatever the robot is doing.
        """
        MyLog.l(self.name, "Stopping manually...")
        if self.random_walking:
            self.stopRandomWalk()
        else:
            self.stopped = True
            
            # if tracking-module is available, stop it
            if self.tracker != None:
                self.tracker.stop()
                
        # stop robots' tracing
        if self.tracer != None:
            self.tracer.stop()
            
    def waitForCompletion(self):
        """
        Wait in calling thread until ePuck finishes what he was doing.
        """
        try:
            if self._thread != None:
                self._thread.join()
            if self.random_walking:
                MyLog.d(self.name, "waitForCompletion() has no effect while random walking.")
        except Exception, pokemon:
            MyLog.e(self.name, "Exception in waitForCompletion: " + pokemon.__str__())
        
        # if tracking-module is available, stop it
        if self.tracker != None:
            self.tracker.stop()

    def getOldOdometry(self):
        """
        Return old odometry (i.e. to draw a line between old and current position).
        """
        return self.old_odometry
   
    def getOdometry(self):
        """
        Return current odometry.
        """
        return self.odometry
    
    def getCorrectionStatus(self):
        """
        Returns True if the robots' current position was corrected by a tracking module.
        """
        return self.is_corrected

    def calcDistance(self, p, q):
        """
        Private. Calculate distance between two given points (Phytagoras).
        """
        return math.sqrt(((p[0] - q[0]) ** 2) + ((p[1] - q[1]) ** 2))
        
    def calcAngleDiff(self, a, b):
        """
        Calculates absolute difference between two angles a(lpha) and b(eta).
        """
        a += 180
        b += 180
        result = ((a - b) ** 2) ** 0.5
        
        if result > 180:
            result = 360 - result
        
        return result
    
    def setCorrectionStatus(self, correction):
        """
        Set self.is_corrected.
        """
        self.is_corrected[0] = correction[0]
        self.is_corrected[1] = correction[1]
