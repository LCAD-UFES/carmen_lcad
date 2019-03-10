
import os
from datasets import experiments
from multiprocessing import Pool

logs_dir = '/media/filipe/Hitachi-Teste/'
data_dir = '/dados/data2/'
program = os.getenv('CARMEN_HOME') + '/src/segmap/graphslam_fast/graphslam'
viewer = 'python ' + os.getenv('CARMEN_HOME') + '/src/segmap/scripts/visualization_and_analysis/generate_graphslam_image.py'
cmds = []


def create_config_file(log_datadir, log, config_file, fused_odom_file):
    f = open(config_file, "w")
    f.write("log:" + log + "\n")
    f.write("loops: none\n")
    f.write("gicp_odom: none\n")
    f.write("gicp_map: none\n")
    f.write("output:" + fused_odom_file + "\n")
    f.write("odom_xy_std: 0.02\n")
    f.write("odom_angle_std: 0.5\n")
    f.write("gps_xy_std: 10.0\n")
    f.write("gps_angle_std: 10.0\n")
    f.close()
    print("Created config file '%s'" % config_file)


view_cmds = []

for e in experiments:
    logs = e['test'] + [e['map']]
    for l in logs:
        log_datadir = data_dir + '/' + 'data_' + l
        log = logs_dir + '/' + l
        config_file = log_datadir + "/config.txt"
        fused_odom_file = log_datadir + "/fused_odom.txt"
        create_config_file(log_datadir, log, config_file, fused_odom_file)
        img_file = fused_odom_file[:-3] + "png"
        cmd = "%s %s" % (program, config_file)
        print(cmd)
        cmds.append(cmd)
        view_cmd = "%s %s %s" % (viewer, fused_odom_file, img_file)
        print(view_cmd)
        print()
        view_cmds.append(view_cmd)


print("Running processes.")    
process_pool = Pool(len(cmds)) 
process_pool.map(os.system, cmds)
process_pool.map(os.system, view_cmds)
print("Done.")

