
import os

base_path = '/dados/experiments/2018_segmap/data_20180907-2/'
data = open(base_path + '/optimized_20180907-2.txt.00.txt', 'r').readlines()

for i in range(len(data)):
    d = data[i]
    d = d.rstrip().rsplit()
    os.system('mv ' + base_path + '/bb3/' + d[5] + '-r.png ' + base_path + '/bb3/%010d.png' % i)
    os.system('mv ' + base_path + '/semantic/seg_' + d[5] + '-r.png ' + base_path + '/semantic/%010d.png' % i)
    os.system('mv ' + base_path + '/velodyne/' + d[4] + '.ply ' + base_path + '/velodyne/%010d.ply' % i)


