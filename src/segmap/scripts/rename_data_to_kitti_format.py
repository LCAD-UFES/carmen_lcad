
import os

base_path = '/dados/data/data_20180112-2/'
data = open(base_path + '/optimized.txt', 'r').readlines()

for i in range(len(data)):
    d = data[i]
    d = d.rstrip().rsplit()
    os.system('cp ' + base_path + '/bb3/' + d[5] + '-r.png ' + base_path + '/bb3/%010d.png' % i)
    os.system('cp ' + base_path + '/semantic/seg_' + d[5] + '-r.png ' + base_path + '/semantic/%010d.png' % i)
    os.system('mv ' + base_path + '/velodyne/' + d[4] + '.ply ' + base_path + '/velodyne/%010d.ply' % i)


