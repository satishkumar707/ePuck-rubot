from math import sin, cos, radians
import numpy as np
import sys

class FileManipulator(object):
    '''
    This program opens a .txt-file, extracts coordinates from its content, rotates
    these coordinates (clockwise) and saves them back (overriding) into the .txt-file.
    '''

    def __init__(self):
        '''
        Constructor
        '''
        pass
    
    def rotateFile2D(self, file_path, center, degree):
        _file = open(file_path, "r")
        
        vectors = []
        
        i = 0
        for line in _file:
            if not i == 0:
                vectors.append([0,0])
                # try reading two numbers from line
                vectors[i - 1][0], vectors[i - 1][1] = line.strip().split()
                        
                vectors[i - 1][0] = int(vectors[i - 1][0])
                vectors[i - 1][1] = int(vectors[i - 1][1])
            i = i + 1
            
        print vectors
        
        _file.close()
        
        vectors_rotated = []
        
        i = 0
        for vector in vectors:
            vectors_rotated.append([0,0])
            rotated = self.rotateVector2D(center, vector, degree)
            vectors_rotated[i][0] = rotated[0]
            vectors_rotated[i][1] = rotated[1]
            i = i + 1
        print vectors_rotated
        
        write_file = open(file_path, "w")
        write_file.write("x\ty\n")
        
        for vector in vectors_rotated:
            write_file.write(str(int(vector[0])) + "\t" + str(int(vector[1])) + "\n")
            
        write_file.close()
    
    def rotateVector2D(self, center, vector, deg):
        rot = np.array([[cos(radians(deg)),-sin(radians(deg))],[sin(radians(deg)),cos(radians(deg))]])
        vector = np.subtract(vector, center)
        r_v = np.dot(rot,vector)
        r_v = np.add(r_v, center)
        return r_v
    
    def rot90(self, c, v):
        return self.rotateVector2D(c,v,90)
    
def main():
    # help
    if '-h' in sys.argv or 'h' in sys.argv or '--help' in sys.argv or 'help' in sys.argv:
        printHelp()
        sys.exit()
        
    path = "path.txt"    
    center = (0,0)
    degree = 90
    
    # parameter
    for i, arg in enumerate( sys.argv ):
        if i == 0: continue
        elif i == 1: path = arg
        elif i == 2: x = int(arg)
        elif i == 3: y = int(arg)
        elif i == 4: degree = int(arg)
    
        
    manipulator = FileManipulator()
    manipulator.rotateFile2D(path, (x,y), degree)

def printHelp():
    print '================================================================================'
    print 'FileManipulator Tool Help                                                              '
    print '--------------------------------------------------------------------------------'
    print 'This program opens a .txt-file, extracts coordinates from its content, rotates'
    print 'these coordinates (clockwise) and saves them back (overriding) into the .txt-file.'
    print '--------------------------------------------------------[ Command Line Parameters ]\n'
    print 'All parameters are mandatory.'
    print '<path>'
    print 'Path of .txt-file containing coordinates. Example: /local/example/path.txt'
    print '<center_x>'
    print 'x-coordinate of origin of rotation. Example: 200'
    print '<center_y>'
    print 'y-coordinate of origin of rotation. Example: 200'
    print '<degree>'
    print 'Coordinates in .txt-file will be rotated <degree> degree.'
    print '--------------------------------------------------------------------[ Examples ]\n'
    print 'Rotate coordinates in ./path.txt 120 degree around point (350,450):'
    print '     $ python FileManipulator.py path.txt 350 450 120'
    print 'Rotate coordinates in /local/example/path.txt 45 degree around point (0,200):'
    print '     $ python FileManipulator.py /local/example/path.txt 0 200 45'
    print '================================================================================'

main()
    
