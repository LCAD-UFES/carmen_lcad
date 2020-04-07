PROG_DESCRIPTION = \
'''
Input: This program reads one or more RDDF files and plots their waypoints on the corresponding map images in a window.
'''
import sys, os, argparse
import numpy as np
import cv2
from PIL import Image
from signal import signal, SIGINT

# Global definitions
WINDOW_NAME = 'RDDF_View'
COLOR = {'cyan':  (0, 255, 255), 'magenta': (255, 0, 255), 'yellow': (255, 255,   0), 'orange': (255, 165, 0), 
         'green': (0, 255,   0), 'red':     (255, 0,   0), 'blue':   (  0,   0, 255)}
COLORS = (COLOR['magenta'], COLOR['green'], COLOR['orange'], COLOR['cyan'], COLOR['red'], COLOR['blue'])


def get_rddf_limits(filelist):
    x_min = y_min = x_max = y_max = None

    for filename in filelist:
        count_points = 0
        f = open(filename)
        for line in f:
            try:
                (x, y) = ( float(val) for val in line.split()[:2] )
                x_min = min(x, x_min) if x_min else x
                y_min = min(y, y_min) if y_min else y
                x_max = max(x, x_max) if x_max else x
                y_max = max(y, y_max) if y_max else y
                count_points += 1
            except ValueError:
                continue
        f.close()
        print('{}RDDF file {} contains {} waypoints'.format('ERROR: ' * (count_points == 0), filename, count_points))
    
    if not x_min:
        print('No valid RDDF files')
        sys.exit(1) 
    
    return (x_min, y_min, x_max, y_max)


def intersection_area(box1, box2):
    inter_area = 0.0
    (x1_min, y1_min, x1_max, y1_max) = box1
    (x2_min, y2_min, x2_max, y2_max) = box2
    x_min = max(x1_min, x2_min)
    y_min = max(y1_min, y2_min)
    x_max = min(x1_max, x2_max)
    y_max = min(y1_max, y2_max)
    if x_max > x_min and y_max > y_min:
        inter_area = float((x_max - x_min) * (y_max - y_min))
    return inter_area


def get_window_limits(imagedir, rddf_limits):
    (x_min, y_min, x_max, y_max) = rddf_limits
    image_list = []

    imagedir_entries = os.listdir(imagedir)
    for entry in imagedir_entries:
        img_filename = entry.split('/')[-1]
        img_filetype = entry.split('.')[-1]
        img_fullpath = imagedir + '/' + img_filename
        if not os.path.isfile(img_fullpath):
            continue
        try:
            coord = ( float(val) for val in img_filename[1:-1 - len(img_filetype)].split('_') )
            (x_low, y_low) = coord
        except ValueError:
            continue
        try:
            img = Image.open(img_fullpath)
            (width, height) = img.size
            img.close()
        except IOError:
            continue
        x_high = x_low + width  * args.scale
        y_high = y_low + height * args.scale
        image_limits = (x_low, y_low, x_high, y_high)
        if intersection_area(image_limits, rddf_limits) == 0.0:
            continue
        image_list.append((img_fullpath, image_limits))
        x_min = min(x_low,  x_min) if x_min else x_low
        y_min = min(y_low,  y_min) if y_min else y_low
        x_max = max(x_high, x_max) if x_max else x_high
        y_max = max(y_high, y_max) if y_max else y_high
        
    window_limits = (x_min, y_min, x_max, y_max)
    show_width  = int(abs((x_max - x_min) / args.scale))
    show_height = int(abs((y_max - y_min) / args.scale))
    print('{} images found: window size {}x{}'.format(len(image_list), show_width, show_height))
    return (image_list, window_limits)


def show_images(image_list, window_limits):
    (x_min, y_min, x_max, y_max) = window_limits
    show_width  = int(abs((x_max - x_min) / args.scale))
    show_height = int(abs((y_max - y_min) / args.scale))
    show_window = np.zeros((show_height, show_width, 3), np.uint8)
    show_window[:] = (255, 255, 255)   # tuple(reversed((30, 144, 255)))  # bluish
    
    for (img_fullpath, image_limits) in image_list:
        img = cv2.imread(img_fullpath)
        (x_low, y_low, x_high, y_high) = image_limits
        x_offset = int(abs((x_low - x_min)  / args.scale))
        y_offset = int(abs((y_max - y_high) / args.scale))
        show_window[y_offset:y_offset + img.shape[0], x_offset:x_offset + img.shape[1]] = img
    
    cv2.imshow(WINDOW_NAME, show_window)
    return show_window


def show_rddf_file(rddf_file, show_window, window_limits):
    global color_index
    color_index = (color_index + 1) % len(COLORS)
    bgr = tuple(reversed(COLORS[color_index]))
    (x_min, y_min, x_max, y_max) = window_limits
    
    rddf = open(rddf_file)
    for waypoint in rddf:
        try:
            (x, y) = ( float(val) for val in waypoint.split()[:2] )
            x_show = int(abs((x - x_min) / args.scale))
            y_show = int(abs((y_max - y) / args.scale))
            cv2.circle(show_window, (x_show, y_show), radius=2, color=bgr, thickness=-1)
        except ValueError:
            continue
    rddf.close()
    cv2.imshow(WINDOW_NAME, show_window)


def shutdown(sig, frame):
    sys.exit(0)
    

def usage_exit(msg = ''):
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
    global parser, args, count_files, total_files, color_index
    signal(SIGINT, shutdown)
    print
    
    parser = argparse.ArgumentParser(description=PROG_DESCRIPTION, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('-i', '--imagedir', help='Image directory   (default: .)', type=_path, default='.')
    parser.add_argument('-s', '--scale', help='Image pixel scale in meters   (default: 0.2)', type=float, default=0.2)
    parser.add_argument('-f', '--filelist', help='text file containing a list of RDDF filenames (one per line)', type=_file)
    parser.add_argument('filename', help='list of RDDF filenames (separated by spaces)', type=_file, nargs='*')
    args = parser.parse_args()
 
    if args.scale <= 0.0:
        usage_exit('Image pixel scale must be a positive value in meters   (default: 0.2): {}\n\n'.format(args.scale))
    
    if not args.filelist and not args.filename:
        if len(sys.argv) > 1:
            usage_exit('At least a filename or a filelist must be passed as argument\n\n')
        usage_exit()
    
    count_files = 0
    total_files = len(args.filename)
    filelist = []
    if args.filelist:
        fl = open(args.filelist)
        filelist = [ f.strip() for f in fl.readlines() if f.strip() and f.strip()[0] != '#' ]
        fl.close()
        total_files += len(filelist)

    rddf_limits = get_rddf_limits(args.filename + filelist)
    (image_list, window_limits) = get_window_limits(args.imagedir, rddf_limits)
    window = show_images(image_list, window_limits)

    color_index = -1
    if args.filename:
        print('********** Processing {} RDDF file{} from commandline'.format(len(args.filename), 's' * (len(args.filename) > 1))) 
        for f in args.filename:
            show_rddf_file(f, window, window_limits)

    if args.filelist:
        print('********** Processing \'{}\' filelist containing {} RDDF file{}'.format(args.filelist, len(filelist), 's' * (len(filelist) > 1))) 
        for f in filelist:
            show_rddf_file(f, window, window_limits)
    
    while (cv2.waitKey(delay=100) & 0xff ) != 27:
        pass

if __name__ == "__main__":
    main()
