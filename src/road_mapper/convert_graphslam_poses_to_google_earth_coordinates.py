'''
Input:  A text file formatted for graphslam_publish
Output: A text file formatted for to be imported by Google Earth
'''
import sys
import argparse
import numpy as np

g_count_lines = 0
g_count_poses = 0


def isfloat(text):
    try:
        float(text)
    except ValueError:
        return False
    return True


def distance(coord1, coord2):
    dist = np.sqrt(np.square(coord1[0] - coord2[0]) + np.square(coord1[1] - coord2[1]))
    return dist


def process_line(line, min_distance, outfile):
    global g_count_lines, g_count_poses
    g_count_lines += 1
    fields = line.split()
    if len(fields) < 4 or not isfloat(fields[0]) or not isfloat(fields[1]):
        sys.stderr.write('Line: %d  Expected format: <lat> <lon> <theta> <timestamp>   Error: %s' %
                         (g_count_lines, line))
        return
    lat = np.fabs(float(fields[0]))
    lon = np.fabs(float(fields[1]))
    if distance((lat, lon), (process_line.last_lat, process_line.last_lon)) >= min_distance:
        outfile.write('%f,%f\n' % (lon, lat))
        g_count_poses += 1
        process_line.last_lat = lat
        process_line.last_lon = lon
    return


def main():
    parser = argparse.ArgumentParser(description='This program reads a text file formatted for graphslam_publish '
                                                 'and creates a text file formatted for to be imported by '
                                                 'Google Earth.',
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('-i', '--infile', help='text file formatted for graphslam_publish',
                        nargs='?', type=argparse.FileType('r'), default=sys.stdin)
    parser.add_argument('-o', '--outfile', help='text file formatted for to be imported by Google Earth',
                        nargs='?', type=argparse.FileType('w'), default=sys.stdout)
    parser.add_argument('-d', '--distance', help='minimum distance between coordinates (meters)',
                        nargs='?', type=float, default=0.5)
    args = parser.parse_args()
    if args.infile == sys.stdin:
        sys.stderr.write('Keyboard input <<<\n')
    process_line.last_lat = 0.0
    process_line.last_lon = 0.0
    for line in args.infile:
        process_line(line, args.distance, args.outfile)
    global g_count_lines, g_count_poses
    sys.stderr.write('%d lines read from %s\n' % (g_count_lines, args.infile.name))
    sys.stderr.write('%d poses written to %s\n' % (g_count_poses, args.outfile.name))
    args.outfile.close()
    return


if __name__ == '__main__':
    main()
