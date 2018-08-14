'''
Input:  A text file formatted for ENet road mapper training
Output: A text file formatted for to be imported by Google Earth
'''
import sys
import argparse

g_count_lines = 0
g_count_poses = 0


def process_line(line, outfile):
    global g_count_lines, g_count_poses
    g_count_lines += 1
    tokens = line.split()
    if len(tokens) != 2:
        sys.stderr.write('Line: %d  Expected format: <filename1> <filename2>   Error: %s' % (g_count_lines, line))
        return
    filename = tokens[0].split('/')[-1]
    fields = filename[1:-4].split('_')
    if len(fields) != 4:
        sys.stderr.write('Line: %d  Expected format: <map_type><lat>_<long>_<off>_<rot>.png   Error: %s\n' %
                         (g_count_lines, filename))
        return
    if float(fields[2]) == 0.0 and float(fields[3]) == 0.0:
        outfile.write('%d,%d\n' % (abs(int(fields[1])), abs(int(fields[0]))))
        g_count_poses += 1
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
    args = parser.parse_args()
    if args.infile == sys.stdin:
        sys.stderr.write('Keyboard input <<<\n')
    for line in args.infile:
        process_line(line, args.outfile)
    global g_count_lines, g_count_poses
    sys.stderr.write('%d lines read from %s\n' % (g_count_lines, args.infile.name))
    sys.stderr.write('%d poses written to %s\n' % (g_count_poses, args.outfile.name))
    args.outfile.close()
    return


if __name__ == '__main__':
    main()
