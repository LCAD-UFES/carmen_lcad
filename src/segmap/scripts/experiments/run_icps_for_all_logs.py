
import os

dirs = [d for d in os.listdir('/dados/data/') if d[:4] == "data"]
for d in dirs:
    d = '/dados/data/' + d 
    cmd = "./gicp_odometry %s %s/odom.txt %s/odom_graphslam.txt" % (d, d, d)
    print('Running ' + cmd) 
    os.system(cmd)
    cmd = "./generate_loop_closures %s %s/loops.txt %s/loops_graphslam.txt" % (d, d, d)
    print('Running ' + cmd) 
    os.system(cmd)

