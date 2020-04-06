PROG_DESCRIPTION = \
'''
Input: This program reads one or more RDDF files and plots their waypoints on the corresponding images of maps.
'''
import sys, os, argparse
import cv2

# Global definitions
COLORS = ((0, 255, 255), (255, 0, 255), (255, 255, 0), (0, 255, 0), (255, 0, 0), (0, 0, 255))







def get_rddf_limits(filelist):
    
    return limits


def get_images(imagedir, scale, limits):
    
    return img


def usage_exit(msg):
    if msg:
        print(msg)
    parser.print_help(sys.stderr)
    sys.exit(1)


def _path(s):
    if not os.path.isdir(s):
        raise argparse.ArgumentTypeError('directory not found: {}'.format(s))
    return s


def _file(s):
    if not os.path.isfile(s):
        raise argparse.ArgumentTypeError('file not found: {}'.format(s))
    return s

    
def main():
    global parser, args, count_files, total_files, img
    print
    
    parser = argparse.ArgumentParser(description=PROG_DESCRIPTION, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('-i', '--imagedir', help='Image directory   (default: .)', type=_path, default='.')
    parser.add_argument('-s', '--scale', help='Image pixel scale in meters   (default: 0.2)', type=float, default=0.2)
    parser.add_argument('-f', '--filelist', help='text file containing a list of RDDF filenames (one per line)', type=_file)
    parser.add_argument('filename', help='list of RDDF filenames (separated by spaces)', type=_file, nargs='*')
    args = parser.parse_args()
 
    if args.scale <= 0.0:
        usage_exit('Image pixel scale must be a positive value in meters: {}\n\n'.format(args.scale))
    
    if not args.filelist and not args.filename:
        if len(sys.argv) > 1:
            usage_exit('At least a filename or a filelist must be passed as argument\n\n')
        usage_exit('')
    
    count_files = 0
    total_files = len(args.filename)
    filelist = []
    if args.filelist:
        fl = open(args.filelist)
        filelist = [ f.strip() for f in fl.readlines() if f.strip() and f.strip()[0] != '#' ]
        fl.close()in mete
        total_files += len(filelist)

    limits = get_rddf_limits(args.filename + filelist)
    img = get_images(args.imagedir, args.scale, limits)

    if args.filename:
        print('********** Processing {} file{} from commandline'.format(len(args.filename), 's' * (len(args.filename) > 1))) 
        for f in args.filename:
            process_rddf_file(f)

    if args.filelist:
        print('********** Processing \'{}\' filelist containing {} file{}'.format(args.filelist, len(filelist), 's' * (len(filelist) > 1))) 
        for f in filelist:
            process_rddf_file(f)

if __name__ == "__main__":
    main()
