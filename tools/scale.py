import sys
import os
import Image

def main():
    sequence_folder = '/local/ePuck/cam/scaled_55/'
    frame_folders   = []
    frame_grab      = 'wide'
    
    # parameter
    for i, arg in enumerate( sys.argv ):
        if i == 0: continue
        elif arg == 'narrow_frame_grab': frame_grab = 'narrow'
        else: frame_folders.append( arg )
        
    if os.path.isdir(sequence_folder) == False:
        try:    os.mkdir(sequence_folder)
        except: print 'Missing destination folder!'; sys.exit()
        
    frame_count = 0
    for folder in frame_folders: frame_count += len(os.listdir(folder))
    print 'No. of frames in all folders:', frame_count
    
    size = Image.open( str(frame_folders[0]+"/"+os.listdir(frame_folders[0])[0]) ).size
    ratio = float(size[0]) / size[1]
    
    width = 0
    if frame_grab == "wide":
        width = 320
    else:
        width = 55
        
    height = int(width / ratio) 
    
    count = 0
    
    for source_folder in frame_folders:

        img_sequence = os.listdir( source_folder )
        img_sequence.sort()

        for source_frame in img_sequence:
            try:
                frame = Image.open( str(source_folder+'/'+source_frame) )
            except:
                print '\nWarning! Skipping inaccessible file', str(source_folder+'/'+source_frame)
                continue

            if frame_grab == 'wide':
                frame.resize( (width, height), Image.ANTIALIAS ).save( sequence_folder+'frame_'+str(count).zfill(5)+'.png' )

            if frame_grab == 'narrow':
                frame.resize( (width, height), Image.ANTIALIAS ).save( sequence_folder+'frame_'+str(count).zfill(5)+'.png' )

            count += 1

            # housekeeping                    
            done = int(count/float(frame_count)*50.0)
            sys.stdout.write( '\r' + '[' + '='*done + '-'*(50-done) + ']~[' + '%.2f' % (count/float(frame_count)*100.0) + '%]' )
            sys.stdout.flush()
    print ''

#-----------------------------------------------------------------------[ Help ]

def printHelp():
                                                             '
    print '--------------------------------------------------------------------------------'
    print 'This program goes through a number of provided folders, assumed to be filled'
    print 'images. They get scaled and stored in a destination folder'
    print 'for further use as generic training material for ratlab.\n'
    print '--------------------------------------------------------[ Command Line Options ]\n'
    print '<list of folders>'
    print '          Each parameter not listed below is assumed to name a folder filled'
    print '          with images to extract image data from.\n'
    print 'narrow_frame_grab'
    print '          By default, the center area (320x40) of each source image is extracted'
    print '          and stored in the destination folder \'sequence_generic\'. If this'
    print '          parameter is set, however, the extracted area is changed to a more'
    print '          narrow window of 55x35 pixels. Both sets of dimensions correspond to'
    print '          the two possible input data formats expected by the ratlab pipeline.\n'
    print '--------------------------------------------------------------------[ Examples ]\n'
    print 'Scale images listed in two test folders:'
    print '     $ python snip.py test_folder_1 test_folder_2'
    print '================================================================================'

main() # <<<   <<<   <<<   <<<   <<<   <<<   <<<   <<<   <<<   <<<   <<<[ main ]
