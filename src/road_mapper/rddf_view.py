#!/usr/bin/env python

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
RGB_COLORS = {'cyan':  (0, 255, 255), 'magenta': (255, 0, 255), 'yellow': (255, 255,   0), 'orange': (255, 165,   0), 
              'green': (0, 255,   0), 'red':     (255, 0,   0), 'blue':   (  0,   0, 255), 'gray':   (230, 230, 230)}
COLOR_LIST = ('magenta', 'green', 'orange', 'cyan', 'red', 'blue', 'gray', 'yellow')
color_index = -1


def get_filelist(filelist_name):
    filelist = []
    if filelist_name:
        fl = open(filelist_name)
        filelist = [ f.strip() for f in fl.readlines() if f.strip() and f.strip()[0] != '#' ]
        fl.close()
    return filelist


def get_rddf_limits(filelist):
    global args
    if args.window:
        (x_min, y_min, width, height) = args.window
        x_max = x_min + width
        y_max = y_min + height
        return (x_min, y_min, x_max, y_max)
    else:
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
    (x1_min, y1_min, x1_max, y1_max) = box1
    (x2_min, y2_min, x2_max, y2_max) = box2
    x_min = max(x1_min, x2_min)
    y_min = max(y1_min, y2_min)
    x_max = min(x1_max, x2_max)
    y_max = min(y1_max, y2_max)
    width  = float(x_max - x_min)
    height = float(y_max - y_min)
    if width <= 0.0 or height <= 0.0:
        return None
    
    x1_off = (x_min - x1_min)
    y1_off = (y_min - y1_min)
    x2_off = (x_min - x2_min)
    y2_off = (y_min - y2_min)
    return (width, height, x1_off, y1_off, x2_off, y2_off)


def get_image_list(imagedir, rddf_limits):
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
        if not intersection_area(rddf_limits, image_limits):
            continue
        image_list.append((img_fullpath, image_limits))
        if not args.window:
            x_min = min(x_low,  x_min) if x_min else x_low
            y_min = min(y_low,  y_min) if y_min else y_low
            x_max = max(x_high, x_max) if x_max else x_high
            y_max = max(y_high, y_max) if y_max else y_high
        
    window_limits = (x_min, y_min, x_max, y_max)
    show_width  = int((x_max - x_min) / args.scale)
    show_height = int((y_max - y_min) / args.scale)
    print('{} images found: window size {}x{}'.format(len(image_list), show_width, show_height))
    return (image_list, window_limits)


def show_images(image_list, window_limits):
    global window_name, args
    (x_min, y_min, x_max, y_max) = window_limits
    show_width  = int((x_max - x_min) / args.scale)
    show_height = int((y_max - y_min) / args.scale)
    show_window = np.zeros((show_height, show_width, 3), np.uint8)
    show_window[:] = (255, 255, 255)   # (30, 144, 255)  # bluish
    
    for (img_fullpath, image_limits) in image_list:
        img = cv2.imread(img_fullpath)
        intersection = intersection_area(window_limits, image_limits)
        if not intersection:
            continue
        (w, h, x_win_off, y_win_off, x_img_off, y_img_off) = ( int(value / args.scale) for value in intersection )
        y_win_off = (show_height  - h) if (y_win_off == 0) else 0
        y_img_off = (img.shape[0] - h) if (y_img_off == 0) else 0
        show_window[y_win_off:y_win_off + h, x_win_off:x_win_off + w] = img[y_img_off:y_img_off + h, x_img_off:x_img_off + w]
    
    window_name = '@{:.0f}_{:.0f}.png'.format(x_min, y_min)
    cv2.imshow(window_name, show_window)
    return show_window


def show_rddf(rddf_file, show_window, window_limits):
    global color_index
    color_index = (color_index + 1) % len(COLOR_LIST)
    bgr_color = tuple(reversed(RGB_COLORS[COLOR_LIST[color_index]]))
    x_min = window_limits[0]
    y_max = window_limits[3]
    
    rddf = open(rddf_file)
    for waypoint in rddf:
        try:
            (x, y) = ( float(val) for val in waypoint.split()[:2] )
            x_show = int((x - x_min) / args.scale)
            y_show = int((y_max - y) / args.scale)
            (window_h, window_w) = show_window.shape[:2]
            if (0 <= x_show < window_w) and (0 <= y_show < window_h):
                cv2.circle(show_window, (x_show, y_show), args.radius, bgr_color, thickness=-1)
        except ValueError:
            continue
    rddf.close()
    cv2.imshow(window_name, show_window)


def shutdown(sig, frame):
    sys.exit(0)
    

def usage_exit(msg = ''):
    global parser
    if msg:
        print(msg)
    parser.print_help(sys.stderr)
    sys.exit(1)


def _dir(s):
    if not os.path.isdir(s):
        raise argparse.ArgumentTypeError('directory not found: {}'.format(s))
    return s


def _file(s):
    if not os.path.isfile(s):
        raise argparse.ArgumentTypeError('file not found: {}'.format(s))
    return s


def read_parameters():
    global parser, args
    parser = argparse.ArgumentParser(description=PROG_DESCRIPTION, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('-i', '--imagedir', help='Image directory   (default: .)', type=_dir, default='.')
    parser.add_argument('-w', '--window', help='Window origin and size in meters: x y width height', type=float, nargs=4)
    parser.add_argument('-s', '--scale', help='Image pixel scale in meters   (default: 0.2)', type=float, default=0.2)
    parser.add_argument('-r', '--radius', help='Waypoint circle radius in pixels   (default: 2)', type=int, default=2)
    parser.add_argument('-f', '--filelist', help='text file containing a list of RDDF filenames (one per line)', type=_file)
    parser.add_argument('filename', help='list of RDDF filenames (separated by spaces)', type=_file, nargs='*')
    args = parser.parse_args()
    
    if len(sys.argv) == 1:
        usage_exit()

    if not args.filelist and not args.filename:
        usage_exit('At least a filename or a filelist must be passed as argument\n\n')

    if args.window and (args.window[2] <= 0.0 or args.window[3] <= 0.0):
        usage_exit('Window size must be two positive values in meters: {}\n\n'.format(args.window[2:]))
    
    if args.scale <= 0.0:
        usage_exit('Image pixel scale must be a positive value in meters   (default: 0.2): {}\n\n'.format(args.scale))
    
    if args.radius < 0:
        usage_exit('Waypoint circle radius must be a non-negative value in pixels   (default: 2): {}\n\n'.format(args.radius))

    
def main():
    global parser, args
    signal(SIGINT, shutdown)
    print

    read_parameters()
    
    filelist = get_filelist(args.filelist)
    rddf_limits = get_rddf_limits(args.filename + filelist)
    (image_list, window_limits) = get_image_list(args.imagedir, rddf_limits)
    window = show_images(image_list, window_limits)
    
    for (f_list, f_list_from) in ((args.filename, '[commandline]'), (filelist, args.filelist)):
        if f_list:
            print("********** Processing {} RDDF file{} from '{}'".format(len(f_list), 's' * (len(f_list) > 1), f_list_from)) 
            for f in f_list:
                show_rddf(f, window, window_limits)
    
    print('Press Esc key on the image window or Ctrl+C on the terminal to finish...\n')
    while (cv2.waitKey(delay=100) & 0xff) != 27:
        pass

if __name__ == '__main__':
    main()
