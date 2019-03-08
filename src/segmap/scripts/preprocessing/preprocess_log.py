
# STEPS BEFORE
# Compile the odometry_calibration module
# Compile the utilities module
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
# rename to kitti format

import os
import sys
import time


def run_cmd(cmd):
    print()
    print(cmd)
    init = time.time()
    output = os.system(cmd)
    print('Output status: ', output, 'Duration:', time.time() - init)
    assert output == 0 or 'graphslam_fast/graphslam' in cmd


def preprocess_log(log_path):
    print('Processing log "%s"' % log_path)
    init = time.time()
    
    if not os.path.exists(log_path):
        print('Error: log "%s" not found. Ignoring it.' % log_path)
        return
    
    carmen_home = os.getenv('CARMEN_HOME') 
    log_name = log_path.rsplit('/')[-1]

    # create output dir if necessary
    output_dir = 'data_' + log_name
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)

    # run odometry calibration
    odom_calib_path = output_dir + '/odom_calib_stderr.txt'
    program_name = carmen_home + '/bin/calibrate_bias_from_log'
    cmd = program_name + ' ' + log_path + ' > ' + output_dir + '/odom_calib_stdout.txt 2> ' + odom_calib_path
    run_cmd(cmd)

    # synchronize messages and store them in a convenient format
    sync_path = '/tmp/sync_' + log_name
    cmd = 'python scripts/step0_parse_and_sync.py ' + log_path + ' > ' + sync_path
    run_cmd(cmd)
    
    # convert gps data to xyz
    sync_xyz_path = output_dir + '/sync.txt' 
    cmd = './scripts/convert_latlon_to_xyz ' + sync_path + ' ' + sync_xyz_path
    run_cmd(cmd)

    # run loop closures
    loops_path = '/tmp/bla.txt'
    #loops_path = output_dir + '/loops.txt'
    #cmd = './graphslam_fast/generate_loop_closures ' + sync_xyz_path + ' ' + loops_path
    #run_cmd(cmd)
    
    # optimize poses
    #cmd = './graphslam_fast/graphslam ' + sync_xyz_path + ' ' + loops_path + ' ' + odom_calib_path + ' ' + output_dir + '/optimized.txt'
    #run_cmd(cmd)

    # convert pointclouds to pcl format
    tmp_velodyne_file = '/tmp/velodynes_' + log_name
    velodyne_dir = output_dir + '/velodyne/'
    if not os.path.exists(velodyne_dir):
        os.mkdir(velodyne_dir)
    run_cmd('grep VELODYNE ' + log_path + ' > ' + tmp_velodyne_file)
    cmd = './scripts/convert_pointclouds_to_pcl ' + tmp_velodyne_file + ' ' + velodyne_dir
    run_cmd(cmd)

    # convert images to png
    tmp_images_path = '/tmp/bbs_' + log_name
    images_dir = output_dir + '/bb3/'
    if not os.path.exists(images_dir):
        os.mkdir(images_dir)
    run_cmd('grep BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE3 ' + log_path + ' > ' + tmp_images_path)
    program_name = carmen_home + '/bin/to_png_new_log'
    cmd = program_name + ' ' + tmp_images_path + ' ' + images_dir + ' -side 1'
    run_cmd(cmd)

    # run semantic segmentation
    semantic_dir = output_dir + '/semantic/'
    if not os.path.exists(semantic_dir):
        os.mkdir(semantic_dir)
    # TODO.
    #cmd = 'cp ' + images_dir + '/* ' + semantic_dir
    #run_cmd(cmd)

    # convert to kitti format
    # cmd = 'python scripts/rename_data_to_kitti_format.py ' + output_dir
    # run_cmd(cmd)

    print('Done. Time elapsed:', time.time() - init)


if __name__ == "__main__":
    args = sys.argv
    if len(args) < 2:
        print("\nError: Use python %s <list of logs>\n" % args[0])
    else:
        logs = args[1:]
        for l in logs:
            preprocess_log(l)


