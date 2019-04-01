
import os
import sys
from experiments import *


def run_odom_calib(carmen_path, log_path, output_dir):
    program = carmen_path + "/src/odometry_calibration/calibrate_bias_from_log"
    output_path = output_dir + "/odom_calib.txt" 
    report_path = output_dir + "/report_odom_calib.txt" 
    cmd = "%s %s %s %s" % (program, log_path, output_path, report_path)
    run_command(cmd)

    # visualization
    program = "python " + carmen_path + "/src/segmap/scripts/visualization_and_analysis/generate_odometry_calibration_image.py"
    img_path = output_dir + "/image_odom_calib.png"
    cmd = "%s %s %s" % (program, report_path, img_path)
    run_command(cmd)


# mode = [fused_odom | graphslam]
def run_graphslam(carmen_path, log_path, output_dir, mode):
    global GPS_XY_STD, GPS_H_STD, LOOPS_XY_STD, LOOPS_H_STD
    
    program = carmen_path + "/src/segmap/graphslam_fast/graphslam"

    if mode == "fused_odom":
        output_path = output_dir + "/fused_odom.txt"
        img_path = output_dir + "/image_fused_odom.png"
        loops = ""
    elif mode == "graphslam":
        output_path = output_dir + "/graphslam.txt"
        img_path = output_dir + "/image_graphslam.png"
        loops = "-l %s/loops.txt" % output_dir
    else:
        raise Exception("Invalid mode '%s'" % mode)
        
    odom_calib = output_dir + "/odom_calib.txt" 
    cmd = "%s %s %s -o %s %s" % (program, log_path, output_path, odom_calib, loops)
    cmd += " --gps_xy_std %lf --gps_angle_std %lf --loop_xy_std %lf --loop_angle_std %lf" % (GPS_XY_STD, GPS_H_STD, LOOPS_XY_STD, LOOPS_H_STD)
    run_command(cmd)

    # visualization
    program = "python " + carmen_path + "/src/segmap/scripts/visualization_and_analysis/generate_graphslam_image.py"
    cmd = "%s %s %s" % (program, output_path, img_path)
    run_command(cmd)


def run_loop_closures(carmen_path, log_path, output_dir):
    global IGNORE_POINTS_ABOVE, IGNORE_POINTS_BELOW, SKIP_WHEN_VELOCITY_IS_BELOW

    program = carmen_path + "/src/segmap/gicp/generate_loop_closures"
    output = output_dir + "/loops.txt"
    odom_calib = output_dir + "/odom_calib.txt" 
    fused_odom = output_dir + "/fused_odom.txt"
    
    cmd = "%s %s %s " % (program, log_path, output)
    cmd += " -o %s -f %s" % (odom_calib, fused_odom)
    cmd += " --ignore_above_threshold %lf --ignore_below_threshold %lf --v_thresh %lf" % (IGNORE_POINTS_ABOVE, IGNORE_POINTS_BELOW, SKIP_WHEN_VELOCITY_IS_BELOW)
    run_command(cmd)


def main(log_path):
    global DATA_DIR

    if not os.path.exists(DATA_DIR):
        os.mkdir(DATA_DIR)

    carmen_path = os.getenv("CARMEN_HOME")     
    output_dir = create_output_dir(log_path)

    run_odom_calib(carmen_path, log_path, output_dir)
    run_graphslam(carmen_path, log_path, output_dir, "fused_odom")     
    run_loop_closures(carmen_path, log_path, output_dir)
    run_graphslam(carmen_path, log_path, output_dir, "graphslam")

    print("Done.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("\nUse python %s <log path>" % sys.argv[0])
        sys.exit(0)
    else:
        main(sys.argv[1])
        
        
        
        
