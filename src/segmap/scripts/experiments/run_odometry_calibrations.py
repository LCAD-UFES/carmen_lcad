
import os, sys
from datasets import experiments
from multiprocessing import Pool


program = os.getenv('CARMEN_HOME') + '/src/odometry_calibration/calibrate_bias_from_log'
viewer = 'python ' + os.getenv('CARMEN_HOME') + '/src/segmap/scripts/visualization_and_analysis/generate_odometry_calibration_image.py'


def run_odometry_calibration(logs_dir, data_dir):
    cmds = []
    viewer_cmds = []

    logs = os.listdir(logs_dir)
    logs = [l for l in logs if (l[:3] == 'log' and l[-3:] == 'txt')]

    for l in logs:
        log_datadir = data_dir + '/' + 'data_' + l
        if not os.path.exists(log_datadir):
            os.mkdir(log_datadir)

        log = logs_dir + '/' + l
        report_file = log_datadir + "/report_odom_calib.txt"
        calib_file = log_datadir + "/odom_calib.txt"

        cmd = "%s %s %s %s" % (program, log, calib_file, report_file)
        print(cmd)
        cmds.append(cmd)

        img_file = report_file[:-3] + "png"
        viewer_cmd = "%s %s %s" % (viewer, report_file, img_file)
        viewer_cmds.append(viewer_cmd)
        print(viewer_cmd)
        print()

    print("Running processes.")    
    process_pool = Pool(len(cmds)) 
    process_pool.map(os.system, cmds)
    process_pool.map(os.system, viewer_cmds)
    print("Done.")
    

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("\nUse python %s <log_dir> <data_dir>")
    else:
        run_odometry_calibration(sys.argv[1], sys.argv[2])
