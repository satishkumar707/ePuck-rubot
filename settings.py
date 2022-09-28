from utils.Freezeable import Freezeable
import math

#====================================================================[ Control ]

# +Setup
# |
# +---+constants
# |   |
# |   +->RAD2DEG:      constant to convert radiant angles into degree angles [def: 180.0/math.pi]
# |
# +---+cam
# |   |
# |   +->track_id:     Tracking-Camera ID. Needed when using multiple cameras [def: 1]
# |   +->robot_id:     Robot-Camera ID. Needed when using multiple cameras [def: 0]
# |   +->norm:         Camera setting. If camera pictures seem wrong, check this setting [def: "PAL"]
# |   +->input:        Camera setting. If camera pictures seem wrong, check this setting [def: "Composite1"]
# |
# +---+filesystem
# |   |
# |   +->file_dir:          Directory where files will be saved [def: "files/"]
# |   +->cam_dir:           Directory where pictures will be saved [def: "cam/"]
# |   +->epuck_dir:         Directory where files from ePucks' navigation will be saved [def: "epuck/"]
# |   +->input_dir:         Directory where input for the program can be found (e.g. path files / SFA network files) [def: "input/"]
# |   +->path_dir:          Directory where path files can be found [def: "path/"]
# |   +->path_file:         Filename of path file (when ePuck is in follow-path-mode) [def: "path.txt"]
# |   +->epuck_follow_path: Full path to path-file [do not change]
# |   +->network_dir:       Directory where network files can be found [def: "...path.../network/"]
# |   +->network_file:      Full path of network file [def: "...path.../filename.tsn"]
# |   +->activity_path:     Directory where cell activity files will be saved [def: "cell_activity/"]
# |
# +---+image
# |   |
# |   +---+tracking
# |   |   |
# |   |   +->offy:         tracking image offset from top [def: 0]
# |   |   +->offx:         tracking image offset from left [def: 70]
# |   |   +->width:        tracking image width [def: 500]
# |   |   +->height:       tracking image height [def: 480]
# |   |
# |   +---+robot
# |       |
# |       +->do_cut:       cut out width_range * height_range from scaled picture [def: True]
# |       +->do_scale:     scale picture to pic_width * pic_height [def: True]
# |       +->width_range:  picture width (format which will be cut out from scaled frame) [def: (83, 633)]
# |       +->height_range: picture height (format which will be cut out from scaled frame) [def: (100, 450)]
# |       +->pic_width:    picture width (format to which will be scaled) [def: 55]
# |       +->pic_height:   picture height (format to which will be scaled) [def: 35]
# |
# +---+robot
# |   |
# |   +->mac:                mac-address of your ePuck [format: "ab:cd:ef:gh:ij:kl"]
# |   +->light_factor:       factor for the lighting of the area (higher values on higher light level). It is used to recognize walls and dodge them with a robot.
# |   |                      If set too low the robot will sense walls which are not there. [def: 1.2]
# |   +->error_threshold     Robots' variable to decide when to ask the tracking-module for a position update (position-dependent). 
# |   |                      0 = always update via tracking-module, e.g. 1 = only update via tracking-module if the error is bigger than once the robots' length. [def: 0.5]
# |   +->angle_err_threshold Robots' variable to decide when to ask the tracking-module for a position update (angle-dependent). 
# |   |                      e.g. 15: If the difference of the angle calculation of the tracking module and the robots' own angle is bigger than 15, update via tracker.
# |   +->diameter:           robots' diameter in mm. [def: 84]
# |
# +---+arena
# |   |
# |   +->markerdist:   maximum allowed distance (pixels) between the center of the robot markers [def: self.robot.diameter]
# |   +->markersize:   calibration marker size in mm [def: 30]
# |   +->boxwidth:     length of the robot box in mm [def: 1000]
# |   +->boxheight:    width of the robot box in mm [def: 1000]
# |
# +---+other
# |   |
# |   +->straps:       store tracked positions in a file [def: True]
# |   +->debug:        debug mode [def: False]
# |   +->filter_log:   You can filter "Log.py"-generated messages in stdout by changing this value to "no/error/log/all"
# |   |                no: there will be no log at all
# |   |                error: Log will only write debug and error messages
# |   |                log: Log will only write log messages
# |   |                [Default] all: Log will write all types of messages
# |
# +---+runparams
#     |5
#     +->correction_mode:      the ePucks navigation can correct its position with help of different modules [def: 1]
#     |                        0: Disable correction
#     |                        1: "Ground Truth". Correct with help of a tracking-module (e.g. overhead camera)
#     |                        2: "Bio Mode". Correct with help of the ePucks' camera and a SFA network.
#     |                        Correction will only be used, if the respective module is activated, too (use_tracking / enable_SFA).
#     +->use_tracking:         if you want to use a tracking-module, set tracking to 1 or True [def: 1]
#     +->new_calibration:      if you want to calibrate the camera with yellow markers, then set calibrate to 1 or True [def: 0]. If it is set to 1, the experiment will not start.
#     +->limit_cond:           limit_cond = ("frames", "time", "path"). Decide when to stop the experiment. 
#     |                        Possibilities: After the cam has collected x frames; after some time has passed; when robot has finished following a path; [def: frames]
#     +->limit_val:            limit_val  = value; frame count or time in sec. Stop experiment after value has been reached. 
#     +->epuck_nav_param:      -1 = ePuck follows path in self.filesystem.epuck_followPath; interval [0, 1] = robot will random walk and [0, 1] determines its momentum [def: 0.5]
#     +->use_cam:              if you want to use the ePucks cam, set use_cam to 1 or True [def: 1]
#     +->init_cam:             initialize norm- and input-mode of ePucks cam. Only initialize once (= run main.py once with init_cam = 1 ). Experiment won't run if this is set to 1. [def: 0]
#     +->cam_start_count:      set starting count for the robots' camera-module. It will save pictures with format "example_000000", starting with cam_start_count [def: 10000]
#     +->enable_SFA:           if set to 1, the given SFA network will be used to collect place cell activity data (when use_cam is set to 1). [def: 0]
#     +->segment_orientation:  cam/setup orientation

# empty utility class
class EmptyOptionContainer(Freezeable):
    def __init__(self):
        pass

# global control panel
class Setup(Freezeable):

    def __init__(self):

        self.constants = EmptyOptionContainer()
        self.constants.RAD2DEG = 180 / math.pi    
        self.constants.freeze()

        self.cam = EmptyOptionContainer()
        self.cam.track_id = 0
        self.cam.robot_id = 1
        self.cam.norm = "PAL"
        self.cam.input = "Composite1"
        self.cam.freeze()

        self.filesystem = EmptyOptionContainer()
        self.filesystem.file_dir = "/local/ePuck/"
        self.filesystem.cam_dir = "cam/"
        self.filesystem.epuck_dir = "epuck/"
        self.filesystem.input_dir = "input/"
        self.filesystem.path_dir = "path/"
        self.filesystem.path_file = "path_c.txt"
        self.filesystem.epuck_follow_path = self.filesystem.input_dir + self.filesystem.path_dir + self.filesystem.path_file
        self.filesystem.network_dir = self.filesystem.input_dir + "network/"
        self.filesystem.network_file = self.filesystem.network_dir + "linC/network_x100000_color_ICA.tsn"
        self.filesystem.activity_path = "cell_activity/"
        self.filesystem.freeze()

        self.image = EmptyOptionContainer()
        self.image.tracking = EmptyOptionContainer()
        self.image.tracking.offy = 0
        self.image.tracking.offx = 0
        self.image.tracking.width = 640
        self.image.tracking.height = 480
        self.image.tracking.freeze()

        self.image.robot = EmptyOptionContainer()
        self.image.robot.do_cut = True
        self.image.robot.do_scale = True
        self.image.robot.width_range = (0, 320)
        self.image.robot.height_range = (100, 140)
        self.image.robot.pic_width = 320
        self.image.robot.pic_height = 240
        self.image.robot.freeze()
        self.image.freeze()

        self.robot = EmptyOptionContainer()
        self.robot.mac = "10:00:E8:C5:61:4B"
        self.robot.light_factor        = 1.1
        self.robot.error_threshold     = 0.3
        self.robot.angle_err_threshold = 1000
        self.robot.diameter            = 84
        self.robot.freeze() 

        self.arena = EmptyOptionContainer()
        self.arena.markerdist = self.robot.diameter
        self.arena.markersize = 35
        self.arena.boxwidth = 1000
        self.arena.boxheight = 310
        self.arena.freeze()

        self.other = EmptyOptionContainer()
        self.other.storepos = False
        self.other.debug = False
        self.other.filter_log = "all"
        self.other.freeze()

        self.runparams = EmptyOptionContainer()
        self.runparams.correction_mode = 1
        self.runparams.use_tracking = 1
        self.runparams.new_calibration = 0
        self.runparams.limit_cond = "time"
        self.runparams.limit_val = 30
        self.runparams.epuck_nav_param = -1
        self.runparams.use_cam = 1
        self.runparams.init_cam = 0
        self.runparams.cam_start_count = 0
        self.runparams.enable_SFA = 1
        self.runparams.segment_orientation = "left_right"
        self.runparams.freeze()

        self.freeze()
