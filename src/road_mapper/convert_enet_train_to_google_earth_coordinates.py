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
        print("Line:", g_count_lines, "  Expected format: <filename1> <filename2>   Error:", line)
        return
    filename = tokens[0].split('/')[-1]
    fields = filename[1:-4].split('_')
    if len(fields) != 4:
        print("Line:", g_count_lines, "  Expected format: <map_type><lat>_<long>_<off>_<rot>.png   Error:", filename)
        return
    if float(fields[2]) == 0.0 and float(fields[3]) == 0.0:
        outfile.write(str(abs(int(fields[1]))) + ',' + str(abs(int(fields[0]))) + '\n')
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
    sys.stderr.write(str(g_count_lines) + ' lines read from ' + args.infile.name + '\n')
    sys.stderr.write(str(g_count_poses) + ' poses written to ' + args.outfile.name + '\n')
    args.outfile.close()
    return


if __name__ == '__main__':
    main()
