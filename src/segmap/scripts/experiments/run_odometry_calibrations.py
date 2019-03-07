
import os
from datasets import experiments
from multiprocessing import Pool

logs_dir = '/media/filipe/Hitachi-Teste/'
data_dir = '/dados/data2/'
program = os.getenv('CARMEN_HOME') + '/src/odometry_calibration/calibrate_bias_from_log'
viewer = 'python ' + os.getenv('CARMEN_HOME') + '/src/segmap/scripts/visualization_and_analysis/generate_odometry_calibration_images.py'
cmds = []

logs = ['log_sao_paulo_brt_20170827-2.txt',
        ]

#for e in experiments:
    #logs = e['test'] + [e['map']]
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

print("Running processes.")    
process_pool = Pool(len(cmds)) 
process_pool.map(os.system, cmds)
os.system(viewer)
print("Done.")