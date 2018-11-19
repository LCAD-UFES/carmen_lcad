
import os

base_path = '/dados/data_20180112-2/data'
data = open(base_path + '/poses.txt', 'r').readlines()

for i in range(len(data)):
    d = data[i]
    d = d.rstrip().rsplit()
    os.system('mv ' + base_path + '/bb3/' + d[5] + '-r.png ' + base_path + '/bb3/%010d.png' % i)
    os.system('mv ' + base_path + '/mobile/' + d[5] + '-r_mobile.png ' + base_path + '/mobile/%010d.png' % i)
    os.system('mv ' + base_path + '/trainfine/' + d[5] + '-r_trainfine.png ' + base_path + '/trainfine/%010d.png' % i)
    os.system('mv ' + base_path + '/velodyne/' + d[4] + '.ply ' + base_path + '/velodyne/%010d.ply' % i)


