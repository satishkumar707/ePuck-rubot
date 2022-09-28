from cv2 import cv
import cv2
import sys

def main(): 
    
    # help
    if '-h' in sys.argv or 'h' in sys.argv or '--help' in sys.argv or 'help' in sys.argv:
        printHelp()
        sys.exit()
    else:
        # cv2 and cv version should be 2.4.5 (and maybe above)
        print "version cv2: " , cv2.__version__
        print "version cv2.cv: " , cv.__version__
    
    cam_id = 0
    
    # parameter
    for i, arg in enumerate( sys.argv ):
        if i == 0: continue
        else: cam_id = arg
    
    cam = cv2.VideoCapture(int(cam_id))
    
    cv2.namedWindow("window", cv.CV_WINDOW_AUTOSIZE)
    
    running = True
    
    while running:
        try:
            flag, img = cam.read()
            if flag:
                cv2.imshow("window", img)
                cv2.waitKey(30)
        except KeyboardInterrupt:
            running = False
        
    cv2.destroyWindow("window")

def printHelp():
    print '================================================================================'
    print 'showCamera Tool Help                                                              '
    print '--------------------------------------------------------------------------------'
    print 'This program opens a camera and displays its frames in real time.'
    print '--------------------------------------------------------[ Command Line Options ]\n'
    print '<camera id>'
    print 'Id of the camera you want to open. If you leave this parameter out default ID 0'
    print 'will be used.'
    print '--------------------------------------------------------------------[ Examples ]\n'
    print 'Show video of camera with id 0 (0 is default):'
    print '     $ python showCameraVideo.py'
    print ' or  $ python showCameraVideo.py 0'
    print 'Show video of camera with id 1 (0 is default):'
    print '     $ python showCameraVideo.py 1'
    print '================================================================================'

main()
