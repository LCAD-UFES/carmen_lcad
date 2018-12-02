
import sys
import os

if __name__ == '__main__':
    args = sys.argv
    if len(args) < 2:
        print('\nError: Use python %s <input_dir>\n' % args[0])
    else:
        base_path = args[1]
        data = open(base_path + '/optimized.txt', 'r').readlines()

        for i in range(len(data)):
            d = data[i]
            d = d.rstrip().rsplit()
            #os.system('cp ' + base_path + '/bb3/' + d[5] + '-r.png ' + base_path + '/bb3/%010d.png' % i)
            #os.system('cp ' + base_path + '/semantic/' + d[5] + '-r.png ' + base_path + '/semantic/%010d.png' % i)
            os.system('cp ' + base_path + '/velodyne/' + d[4] + '.ply ' + base_path + '/velodyne/%010d.ply' % i)


