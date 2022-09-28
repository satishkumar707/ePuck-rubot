from utils.Freezeable import Freezeable
from utils import Log as MyLog
from time import sleep
from threading import Thread
from settings import Setup
import random

class randomVehicle(Freezeable):
    """
    This class implements a random walk.
    """


    def __init__(self, ePuckControl):
        """
        Constructor
        """
        # setup variables
        self.setup = Setup()
        self.name = "randomVehicle"
        self.epuck = ePuckControl
        
        # monitors the status of this class' thread
        self.active = False
        
        # initialize generator with system time
        random.seed()
        
        # threshold for summed proximity
        self.threshold_close = 60 * self.setup.robot.light_factor
        # threshold for single proximity
        self.threshold_single_close = 75 * self.setup.robot.light_factor
        
        # minimal and maximal wheel speed
        self.min_speed = 300
        self.max_speed = 900
        # maximum change in wheel speed
        self.delta_limit = 100
        
        # standard speed after dodging a wall
        self.standard_speed = 800
        
        # turning speed when dodging walls
        self.dodge_turning_speed = 400
        
        # tuple of proximity values
        self.proximity = None
        
        # update motor speed only every x steps
        self.run = 10
        
        # when dodging wall, continue turning x steps, to have a greater angle to the wall
        self.continue_turning_steps = 2
        self.continue_turning = self.continue_turning_steps
        
        # monitors if the robot is dodging a wall
        self.dodging_wall = False
        self.just_dodged_wall = False
        
        self.freeze()
        
        # enable sensors
        self.epuck.enable("proximity")
        self.epuck.enable("motor_speed")
        
    def start(self, momentum):
        """
        Start random walk in its own thread.
        """
        self.active = True
        try:
            MyLog.l(self.name, "Starting thread: randomWalk(" + str(momentum) + ").")
            self._thread = Thread(target=self.threadedRandomWalk, args=[momentum])
            self._thread.start()
        except Exception, pokemon:
            MyLog.e(self.name, "Exception in start: " + pokemon.__str__())
    
    def stop(self):
        """
        Stop random walk and its thread.
        """
        self.active = False
        
    def threadedRandomWalk(self, momentum):
        """
        Stable random walk method.
        """
        motor_speed = [0, 0]
        counter = 0
        while self.active:
            try:
                self.epuck.step()
            except Exception, pokemon:
                MyLog.e(self.name, pokemon)
            self.updateProximity()
            
            # for tracing purposes
            self.epuck.updatePosition()
            
            # weighted sum of proximity values
            sumLeft = 0.05 * self.proximity[5] + 0.4 * self.proximity[6] + 0.55 * self.proximity[7]
            sumRight = 0.05 * self.proximity[2] + 0.4 * self.proximity[1] + 0.55 * self.proximity[0]
            
            #print self.proximity, sumLeft, "...", sumRight
            
            ### wall detection ###
            # if summed sensors > threshold or single sensor > threshold
            if (sumLeft > self.threshold_close or sumRight > self.threshold_close or self.proximity[7] > self.threshold_single_close or self.proximity[6] > self.threshold_single_close or self.proximity[0] > self.threshold_single_close or self.proximity[1] > self.threshold_single_close):
                # if left proximity values > right proximity values
                if sumLeft > sumRight:
                    # turn right on the spot
                    motor_speed[0] = self.dodge_turning_speed
                    motor_speed[1] = -self.dodge_turning_speed
                else:
                    # turn left on the spot
                    motor_speed[0] = -self.dodge_turning_speed
                    motor_speed[1] = self.dodge_turning_speed
                    
                if not self.dodging_wall:
                    try:
                        # reset continue_turning, so that the robot continues turning away from wall
                        self.continue_turning = self.continue_turning_steps
                        
                        # randomize continue_turning
                        random_add = 1
                        if random.random() >= 0.5:
                            if random.random() >= 0.5:
                                if random.random() >= 0.5:
                                    self.continue_turning = self.continue_turning + random_add
                                else:
                                    self.continue_turning = self.continue_turning + random_add - 1
                            else:
                                self.continue_turning = self.continue_turning + random_add -2
                        else:
                            if random.random() >= 0.5:
                                if random.random() >= 0.5:
                                    self.continue_turning = self.continue_turning - random_add
                                else:
                                    self.continue_turning = self.continue_turning - random_add + 1
                            else:
                                self.continue_turning = self.continue_turning - random_add + 2
                            
                        # send motor speed to robot
                        self.epuck.set_motors_speed(motor_speed[0], motor_speed[1])
                        self.epuck.step()
                    except Exception, pokemon:
                        MyLog.e(self.name, pokemon)
                    
                self.just_dodged_wall = False
                self.dodging_wall = True
            else:
                if self.continue_turning > 0:
                    # continue turning
                    self.continue_turning = self.continue_turning - 1
                else:
                    # stop turning
                    self.dodging_wall = False
                    
            # continue with standard speed after dodging wall
            if not self.dodging_wall and not self.just_dodged_wall:
                self.just_dodged_wall = True
                self.epuck.set_motors_speed(self.max_speed, self.max_speed)
                try:
                    self.epuck.step()
                except Exception, pokemon:
                    MyLog.e(self.name, pokemon)

            ### random wheel speed update ###
            # only update wheel speeds if (some time has passed AND not dodging a wall)
            if counter % self.run == 0 and not self.dodging_wall :
                # decides whether left or right wheel gets noise (number in [0.0, 1.0))
                chance = random.random()
                noise = random.random() * self.max_speed
                
                if chance >= 0.5:
                    # => left wheel slows down
                    motor_speed[0] = momentum * self.max_speed + (1 - momentum) * noise
                    # max speed
                    motor_speed[1] = self.max_speed
                else:
                    # => right wheel slows down
                    motor_speed[0] = self.max_speed
                    motor_speed[1] = momentum * self.max_speed + (1 - momentum) * noise
                    
                # set wheel speed
                try:
                    self.epuck.set_motors_speed(motor_speed[0],motor_speed[1])
                    self.epuck.step()
                except Exception, pokemon:
                    MyLog.e(self.name, "Exception in randomWalk: " + pokemon.__str__())  
            
            counter = counter + 1
            # wait 5 ms, so other threads have time to work
            sleep(0.005)
            
        # loop stopped, stop the robot
        motor_speed = [0, 0]
        
        try:
            self.epuck.set_motors_speed(motor_speed[0],motor_speed[1])
            self.epuck.step()
        except Exception, pokemon:
            MyLog.e(self.name, "Exception in randomWalk: " + pokemon.__str__())    
    
    def updateProximity(self):
        """
        Private. Requests proximity values of the robot.
        """
        try:
            prox = self.epuck.get_proximity()
            self.proximity = list(prox)
            
            # fix proximity values
            for i in range(0, 8):
                if self.proximity[i] > 65000:
                    self.proximity[i] = 10
        
        except Exception, pokemon:
            MyLog.e(self.name, "Exception in updateProximity: " + pokemon.__str__())
