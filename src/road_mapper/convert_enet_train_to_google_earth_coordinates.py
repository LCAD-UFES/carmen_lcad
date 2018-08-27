'''
Input:  A text file formatted for ENet road mapper training
Output: A text file formatted for to be imported by Google Earth
'''
import sys
import argparse
import numpy as np

g_count_lines = 0
g_count_poses = 0


def isfloat(var):
    if type(var).__name__ in ('list', 'tuple'):
        for element in var:
            if not isfloat(element):
                return False
    else:
        try:
            float(var)
        except ValueError:
            return False
    return True


def distance(coord1, coord2):
    (x1, y1), (x2, y2) = (coord1, coord2)
    dist = np.sqrt(np.square(x2 - x1) + np.square(y2 - y1))
    return dist


def process_line(line, min_distance, outfile):
    global g_count_lines, g_count_poses
    g_count_lines += 1
    tokens = line.split()
    if len(tokens) != 2:
        sys.stderr.write('Line: %d  Expected format: <filename1> <filename2>   Error: %s' % (g_count_lines, line))
        return
    filename = tokens[0].split('/')[-1]
    fields = filename[1:-4].split('_')
    if len(fields) != 4 or not isfloat(fields):
        sys.stderr.write('Line: %d  Expected format: <map_type><lat>_<lon>_<off>_<rot>.png   Error: %s\n' %
                         (g_count_lines, filename))
        return
    lat, lon, off, rot = (np.fabs(float(fields[i])) for i in range(4))
    if off == 0.0 and rot == 0.0 and \
            distance((lat, lon), (process_line.last_lat, process_line.last_lon)) >= min_distance:
        outfile.write('%f,%f\n' % (lon, lat))
        g_count_poses += 1
        process_line.last_lat, process_line.last_lon = (lat, lon)
    return


def main():
    parser = argparse.ArgumentParser(description='This program reads a text file formatted for ENet road mapper '
                                                 'training and creates a text file formatted for to be imported by '
                                                 'Google Earth.',
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('-i', '--infile', help='text file formatted for ENet road mapper training',
                        nargs='?', type=argparse.FileType('r'), default=sys.stdin)
    parser.add_argument('-o', '--outfile', help='text file formatted for to be imported by Google Earth',
                        nargs='?', type=argparse.FileType('w'), default=sys.stdout)
    parser.add_argument('-d', '--distance', help='minimum distance between coordinates (meters)',
                        nargs='?', type=float, default=5.0)
    args = parser.parse_args()
    if args.infile == sys.stdin:
        sys.stderr.write('Standard input <<<\n')
    process_line.last_lat, process_line.last_lon = (0.0, 0.0)
    for line in args.infile:
        process_line(line, args.distance, args.outfile)
    global g_count_lines, g_count_poses
    sys.stderr.write('%d lines read from %s\n' % (g_count_lines, args.infile.name))
    sys.stderr.write('%d poses written to %s\n' % (g_count_poses, args.outfile.name))
    args.outfile.close()
    return


if __name__ == '__main__':
    main()
