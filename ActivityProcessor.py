from utils.Freezeable import Freezeable
from settings import Setup
from utils import Log as MyLog
import Image as im
import numpy as np
import sys
import os
import pickle

class ActivityProcessor(Freezeable):
    """
    This class can read activity data files (created by PlaceCellCalculation), 
    manipulate their data and save it to pictures.
    There are three possibilities when running this module:
    1. Save raw activity pictures (set variable mode to "raw" at the bottom of this file).
    2. Save linear rasterized activity pictures (set mode to "raster")
    3. Save both, raw and linear rasterized activity pictures (set mode to "both")
    
    Variable "segments" (at the bottom of this file) determines in how many segments 
    the picture will be rasterized when running in "raster"- or "both"-mode.
    """

    def __init__(self, global_limit, l_limit, r_limit):
        '''
        Constructor
        '''
        self.setup = Setup()
        self.name = "ActivityProcessor"
        
        self.limit = global_limit
        self.l_limit = l_limit
        self.r_limit = r_limit
        
        self.freeze()
        
    def unpickleData(self, path):
        return pickle.load(open(path, "rb"))
    
    def drawRawActivityPics(self, data):
        """
        Find min/max values over all positions for each cell,
        convert SFA-data with a min/max-scaled color scale,
        write scaled data into images and write images to files.
        
        FOR all cells (32):
            FOR all positions(x,y):
                GET min/max values
            FOR all positions(x,y):
                GET min/max scaled color values
                STORE color values in image
        """
        MyLog.l(self.name, "Drawing cell activity pictures...")
        
        # example: _min[0][0] gives minimum for cell 0 in left direction.
        #          _min[0][1] gives minimum for cell 0 in right direction.
        _min = []
        _max = []
        
        path = self.setup.filesystem.file_dir + self.setup.filesystem.activity_path + "raw/"   
        
        for i in range(0, 32):
            # prepare images
            sample_img_left = im.new('RGBA'
                                     , (self.setup.arena.boxwidth
                                        , self.setup.arena.boxheight))
            sample_data_left = sample_img_left.load()
            sample_img_right = im.new('RGBA'
                                      , (self.setup.arena.boxwidth
                                         , self.setup.arena.boxheight))
            sample_data_right = sample_img_right.load()
            
            # append minimum and maximum to corresponding list
            _min.append([np.amin(data[:, :, i, 0]), np.amin(data[:, :, i, 1])])
            _max.append([np.amax(data[:, :, i, 0]), np.amax(data[:, :, i, 1])])
        
            # convert data color values with colorScale
            for x in range(0, self.setup.arena.boxwidth):
                for y in range(0, self.setup.arena.boxheight):
                    # apply color scale on left and right data and write into image
                    sample_data_left[x, y] = self.colorScale_ZA(data[x, y, i, 0], _min[i][0], _max[i][0])
                    sample_data_right[x, y] = self.colorScale_ZA(data[x, y, i, 1], _min[i][1], _max[i][1])
                    
            self.saveImage(sample_img_left, path + "left/", "activity_left_" + str(i) + ".png")
            self.saveImage(sample_img_right, path + "right/", "activity_right_" + str(i) + ".png")
            
        MyLog.l(self.name, "Drawing cell activity pictures finished.")
        
    def colorScale_ZA(self, value, val_min, val_max, zero_level=127, scalar_only=False):  # ZA: zero align (preset zero_level in [0,1024])
        """
        Scale value between val_min and val_max and return its color-tuple.
        """
        # setup
        pos_scale = 1024.0 - zero_level
        if val_max == 0:
            return (0, 0, 128, 255)
        else:
            scale_factor = pos_scale / val_max
        
        if value < val_min: value = val_min
        elif value > val_max: value = val_max
        
        scale_value = int(zero_level + (value * scale_factor))
        
        if scalar_only == True: return (scale_value if scale_value >= 0 else 0)
        # cutoff smaller values
        if scale_value < 0: scale_value = 0
        
        # scale value to RGB color
        rgb = np.array([0, 0, 0])
        
        if(scale_value < 128):
            rgb[2] = 128 + scale_value
        elif(scale_value < 384):
            rgb[1] = scale_value - 128
            rgb[2] = 255
        elif(scale_value < 640.0):
            rgb[0] = scale_value - 384
            rgb[1] = 255
            rgb[2] = 255 - (scale_value - 384)
        elif(scale_value < 896):
            rgb[0] = 255
            rgb[1] = 255 - (scale_value - 640)
        else:
            rgb[0] = 255 - (scale_value - 896)
            
        # return RGBA color tuble
        return (rgb[0], rgb[1], rgb[2], 255)
    
    def printValues(self, data):
        """
        Prints activity data which is not 0, so that you can
        find out in which range the activity values are.
        """
        count = 0
        for i in range(0,32):
            for x in range(0,self.setup.arena.boxwidth):
                for y in range(0,self.setup.arena.boxheight):
                    for z in range(0,2):
                        if data[x,y,i,z] != 0:
                            if z == 0:
                                print "left:", data[x,y,i,z]
                            else:
                                print "right:", data[x,y,i,z]
                            count += 1
        print count
        print self.setup.arena.boxheight * self.setup.arena.boxwidth
        
    def drawRasterizedActivityPics(self, segments, data):
        """
        Rasterize data and draw it.
        """
        # data validation
        if data == None:
            raise Exception("data is None!")
        
        if segments <= 0:
            raise Exception("Segments is <= 0. Should be bigger than 0!")
        
        MyLog.l(self.name, "Drawing rasterized cell activity pictures...")
          
        _min = []  
        _max = []
        global_min = []
        global_max = []
        
        # uncomment to see the values of the activity data (so you can adapt "limit" at the bottom of this file)
        # self.printValues(data)three possibilities when running this module:
        
        path = self.setup.filesystem.file_dir + self.setup.filesystem.activity_path + "rasterized_" + str(segments) + "/"   
        
        for i in range(0, 32):
            avg_activity = []
            
            # compute width of one segment
            if self.setup.runparams.segment_orientation == "left_right":
                segment_width = int(self.setup.arena.boxwidth / segments)
            elif self.setup.runparams.segment_orientation == "up_down":
                segment_width = int(self.setup.arena.boxheight / segments)
            
            if segment_width == 0:
                raise Exception("Too many segments, segments_width is 0 pixel! Restart with less segments.")
            
            # compute width difference which forms through approximating with int()
            if self.setup.runparams.segment_orientation == "left_right":
                diff = self.setup.arena.boxwidth - segment_width * segments
                # prepare images
                sample_img_left = im.new('RGBA'
                                         , (self.setup.arena.boxwidth - diff
                                            , self.setup.arena.boxheight))
                sample_data_left = sample_img_left.load()
                sample_img_right = im.new('RGBA'
                                          , (self.setup.arena.boxwidth - diff
                                             , self.setup.arena.boxheight))
                sample_data_right = sample_img_right.load()
            
            elif self.setup.runparams.segment_orientation == "up_down":
                diff = self.setup.arena.boxheight - segment_width * segments                
                # prepare images
                sample_img_left = im.new('RGBA'
                                         , (self.setup.arena.boxwidth 
                                            , self.setup.arena.boxheight - diff))
                sample_data_left = sample_img_left.load()
                sample_img_right = im.new('RGBA'
                                          , (self.setup.arena.boxwidth
                                             , self.setup.arena.boxheight - diff))
                sample_data_right = sample_img_right.load()
                
            # find the average activity value for every segment
            for z in range(0, segments):
                # reset activity and count for current cell
                activity = [0, 0]
                count = [0, 0]
                
                # find the average for current segment in left and right picture
                if self.setup.runparams.segment_orientation == "left_right":
                    for x in range(0, segment_width):
                        for y in range(0, self.setup.arena.boxheight):
                            if data[x + z * segment_width, y, i, 0] != 0:
                                # cut data-value if it exceeds limit
                                if data[x + z * segment_width, y, i, 0] > self.limit:
                                    data[x + z * segment_width, y, i, 0] = self.limit
                                elif data[x + z * segment_width, y, i, 0] < -self.limit:
                                    data[x + z * segment_width, y, i, 0] = -self.limit
                                    
                                activity[0] += data[x + z * segment_width, y, i, 0]
                                count[0] += 1
                            if data[x + z * segment_width, y, i, 1] != 0:
                                # cut data-value if it exceeds limit
                                if data[x + z * segment_width, y, i, 1] > self.limit:
                                    data[x + z * segment_width, y, i, 1] = self.limit
                                elif data[x + z * segment_width, y, i, 1] < -self.limit:
                                    data[x + z * segment_width, y, i, 1] = -self.limit
                                    
                                activity[1] += data[x + z * segment_width, y, i, 1]
                                count[1] += 1
                                
                elif self.setup.runparams.segment_orientation == "up_down":
                    for y in range(0, segment_width):
                        for x in range(0, self.setup.arena.boxwidth):
                            if data[x, y + z * segment_width, i, 0] != 0:
                                # cut data-value if it exceeds limit
                                if data[x, y + z * segment_width, i, 0] > self.limit:
                                    data[x, y + z * segment_width, i, 0] = self.limit
                                elif data[x, y + z * segment_width, i, 0] < -self.limit:
                                    data[x, y + z * segment_width, i, 0] = -self.limit
                                    
                                activity[0] += data[x, y + z * segment_width, i, 0]
                                count[0] += 1
                            if data[x, y + z * segment_width, i, 1] != 0:
                                # cut data-value if it exceeds limit
                                if data[x, y + z * segment_width, i, 1] > self.limit:
                                    data[x, y + z * segment_width, i, 1] = self.limit
                                elif data[x, y + z * segment_width, i, 1] < -self.limit:
                                    data[x, y + z * segment_width, i, 1] = -self.limit
                                    
                                activity[1] += data[x, y + z * segment_width, i, 1]
                                count[1] += 1
                            
                # compute average
                try:
                    activity[0] /= count[0]
                except ZeroDivisionError:
                    activity[0] = 0
                try:
                    activity[1] /= count[1]
                except ZeroDivisionError:
                    activity[1] = 0
                
                # append list of average activity of left and right image of current segment to average-list (which will be drawn later on)
                avg_activity.append([activity[0], activity[1]])
                
            # append minimum of left and right activity data to corresponding list, same applies for maximum
            _min.append([np.amin(data[:, :, i, 0]), np.amin(data[:, :, i, 1])])
            _max.append([np.amax(data[:, :, i, 0]), np.amax(data[:, :, i, 1])])
            
            # append global minimum of left and right activity data to corresponding list, same applies for maximum
            global_min.append(np.amin(data[:, :, i, :]))
            global_max.append(np.amax(data[:, :, i, :]))
                
            #print avg_activity
            color = [(0, 0, 0, 0), (0, 0, 0, 0)]
            
            # now draw avg_activity for global minima and maxima
            for z in range(0, segments):
                # get color for current segment for left and right image
                color[0] = self.colorScale_ZA(avg_activity[z][0], global_min[i], global_max[i])
                color[1] = self.colorScale_ZA(avg_activity[z][1], global_min[i], global_max[i])
                if self.setup.runparams.segment_orientation == "left_right":
                    for x in range(0, segment_width):
                        for y in range(0, self.setup.arena.boxheight):
                            # draw current segment for left and right picture
                            sample_data_left[x + z * segment_width, y] = color[0]
                            sample_data_right[x + z * segment_width, y] = color[1]
                elif self.setup.runparams.segment_orientation == "up_down":
                    for y in range(0, segment_width):
                        for x in range(0, self.setup.arena.boxwidth):
                            # draw current segment for left and right picture
                            sample_data_left[x, y + z * segment_width] = color[0]
                            sample_data_right[x, y + z * segment_width] = color[1]
            try:
                if self.setup.runparams.segment_orientation == "left_right":
                    self.saveImage(sample_img_left, path + "global_" + str(self.limit) + "/left/", "activity_left_" + str(i) + ".png")
                    self.saveImage(sample_img_right, path + "global_" + str(self.limit) +"/right/", "activity_right_" + str(i) + ".png") 
                elif self.setup.runparams.segment_orientation == "up_down":
                    self.saveImage(sample_img_left, path + "global_" + str(self.limit) + "/down/", "activity_down_" + str(i) + ".png")
                    self.saveImage(sample_img_right, path + "global_" + str(self.limit) +"/up/", "activity_up_" + str(i) + ".png")
            except:
                MyLog.e(self.name, "Exception trying to save an image!")
                
            # now draw avg_activity for local minima and maxima
            for z in range(0, segments):
                # get color for current segment for left and right image
                if _max[i][0] > self.l_limit: _max[i][0] = self.l_limit
                if _max[i][1] > self.r_limit: _max[i][1] = self.r_limit
                if _min[i][0] < -self.l_limit: _min[i][0] = -self.l_limit
                if _min[i][1] < -self.r_limit: _min[i][1] = -self.r_limit
                color[0] = self.colorScale_ZA(avg_activity[z][0], _min[i][0], _max[i][0])
                color[1] = self.colorScale_ZA(avg_activity[z][1], _min[i][1], _max[i][1])

                if self.setup.runparams.segment_orientation == "left_right":
                    for x in range(0, segment_width):
                        for y in range(0, self.setup.arena.boxheight):
                            # draw current segment for left and right picture
                            sample_data_left[x + z * segment_width, y]  = color[0]
                            sample_data_right[x + z * segment_width, y] = color[1]
                elif self.setup.runparams.segment_orientation == "up_down":
                    for y in range(0, segment_width):
                        for x in range(0, self.setup.arena.boxwidth):
                            # draw current segment for left and right picture
                            sample_data_left[x, y + z * segment_width]  = color[0]
                            sample_data_right[x, y + z * segment_width] = color[1]
            try:
                if self.setup.runparams.segment_orientation == "left_right":
                    self.saveImage(sample_img_left, path + "local/left_" + str(self.l_limit) + "/", "activity_left_" + str(i) + ".png")
                    self.saveImage(sample_img_right, path + "local/right_" + str(self.r_limit) + "/", "activity_right_" + str(i) + ".png")
                elif self.setup.runparams.segment_orientation == "up_down":
                    self.saveImage(sample_img_left, path + "local/down_" + str(self.l_limit) + "/", "activity_down_" + str(i) + ".png")
                    self.saveImage(sample_img_right, path + "local/up_" + str(self.r_limit) + "/", "activity_up_" + str(i) + ".png")
            except:
                MyLog.e(self.name, "Exception trying to save an image!")
        
        print "local minima for every cell:", _min
        print "local maxima for every cell:", _max
        print "Minima for every cell:", global_min
        print "Maxima for every cell", global_max
        
        MyLog.l(self.name, "Drawing rasterized cell activity pictures finished.")
    
    def saveImage(self, im, path, file_name):
        """
        Save given image to file.
        """
        # save image to file
        try:
            im.save(path + file_name)
        except IOError:
            if not os.path.exists(path):
                # create directories
                os.makedirs(path)
                
                # try saving files again
                im.save(path + file_name)
   
def printHelp():
    print '================================================================================'
    print 'ActivityProcessor Help                                                              '
    print '--------------------------------------------------------------------------------'
    print 'This program loads an activity data file and draws it.'
    print 'Check path, segments, global_limit, l_limit and r_limit at the bottom of the file.'
    print '--------------------------------------------------------[ Command Line Options ]\n'
    print '<raw>'
    print '          ActivityProcessor will only draw the raw data of your activity '
    print '          data file.'
    print '<raster>'
    print '          ActivityProcessor will only draw the rasterized data of your file.'
    print '<both>'
    print '          You do not have to add this paramter explicitely. It is set as default.'
    print '          ActivityProcessor will draw the raw and rasterized data of your file.\n'
    print '--------------------------------------------------------------------[ Examples ]\n'
    print 'Only draw rasterized data:'
    print '     $ python ActivityProcessor.py raster'
    print 'Only draw raw data:'
    print '     $ python ActivityProcessor.py raw'
    print '================================================================================' 
    
if __name__ == "__main__":
    
    # help
    if '-h' in sys.argv or 'h' in sys.argv or '--help' in sys.argv or 'help' in sys.argv:
        printHelp()
        sys.exit()
    
    # path to SFA activity data
    path = "/local/ePuck/cell_activity/cell_data.dat"
    
    # determines what will be done when executing this
    mode = "both"

    # number of segments when drawing rasterized activity data
    segments = 100
    
    # this variable is used when rasterizing activity data.
    # The average value of a segment will be limited to [-self.global_limit, self.global_limit],
    # effecting the outcome of the global minima/maxima pictures.
    # Same applies to local minima/maxima pictures, but the variables here are l_limit (limit for left pictures)
    # and r_limit (limit for right pictues)
    global_limit = 5
    l_limit = 4
    r_limit = 3
    
    activity_processor = ActivityProcessor(global_limit, l_limit, r_limit)
    
    # process console input
    for i, arg in enumerate(sys.argv):
        if i == 0: continue
        elif arg == "raw": mode = arg
        elif arg == "raster": mode = arg
        elif arg == "both": mode = arg
        else: path = arg

    # draw raw activity pictures
    if mode == "raw":
        activity_processor.drawRawActivityPics(activity_processor.unpickleData(path))
        
    # draw rasterized activity pictures    
    elif mode == "raster":
        activity_processor.drawRasterizedActivityPics(segments, activity_processor.unpickleData(path))
    
    # draw raw and rasterized activity pictures
    elif mode == "both":
        activity_processor.drawRawActivityPics(activity_processor.unpickleData(path))
        activity_processor.drawRasterizedActivityPics(segments, activity_processor.unpickleData(path))
