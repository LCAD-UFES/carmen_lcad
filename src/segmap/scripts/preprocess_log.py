
# STEPS BEFORE
# Compile the odometry_calibration module
# Compile the segmap module

# HOW TO GENERATE DATA DIRS
# calibrate odometry 
# convert images to png
# run semantic segmentation for the png images
# synchronize
# convert latlong to xyz  
# run loop closures (optional)
# optimize
# convert pointclouds to ply
# create the following data dir


import os
import sys

def preprocess_log(log_path):
    if not os.path.exists(log_path):
        print('Error: log "%s" not found. Ignoring it.' % log_path)
        return
    
    carmen_home = os.getenv('CARMEN_HOME') 
    log_name = log_path.rsplit('/')[-1]

    # create output dir
    output_dir = 'data_' + log_name
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)

    program_name = carmen_home + '/bin/calibrate_bias_from_log'
    cmd = program_name + ' ' + log_path + ' > ' + output_dir + '/odom_calib_stdout.txt 2> ' + output_dir + '/odom_calib_stderr.txt'
    print(cmd)
    output = os.system(cmd)
    print('Output status: ', output)
    
    
    

if __name__ == "__main__":
    args = sys.argv
    if len(args) < 2:
        print("\nError: Use python %s <list of logs>\n" % args[0])
    else:
        logs = args[1:]
        for l in logs:
            preprocess_log(l)
