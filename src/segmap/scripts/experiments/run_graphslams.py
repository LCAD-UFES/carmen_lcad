
import os, sys
from datasets import experiments
from multiprocessing import Pool

program = os.getenv('CARMEN_HOME') + '/src/segmap/graphslam_fast/graphslam'
viewer = 'python ' + os.getenv('CARMEN_HOME') + '/src/segmap/scripts/visualization_and_analysis/generate_graphslam_image.py'


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


def run_graphslams(logs_dir, data_dir):
    cmds = []
    view_cmds = []

    logs = os.listdir('/dados/logs')
    logs = [l for l in logs if (l[:3] == 'log' and l[-3:] == 'txt')]
    logs_dir = '/dados/logs'

    for l in logs:
        log_datadir = data_dir + '/' + 'data_' + l
        log = logs_dir + '/' + l
        config_file = log_datadir + "/config_fused_odom.txt"
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


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("\nUse python %s <log_dir> <data_dir>")
    else:
        run_graphslams(sys.argv[1], sys.argv[2])
