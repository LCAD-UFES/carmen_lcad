
import os
import sys
import argparse
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
		loops = "-l %s/loops.txt --pf_loops %s/localization_loops.txt" % (output_dir, output_dir)
	else:
		raise Exception("Invalid mode '%s'" % mode)
		
	odom_calib = output_dir + "/odom_calib.txt" 
	cmd = "%s %s %s -o %s %s" % (program, log_path, output_path, odom_calib, loops)
	cmd += " --gps_xy_std 2.000000 --gps_angle_std 20.000000 --gicp_loops_xy_std 0.300000 --gicp_loops_angle_std 1.000000 --pf_loops_xy_std 0.020000 --pf_loops_angle_std 3.000000 --gps_discontinuity_threshold 0.5 --gps_min_cluster_size 50"
	run_command(cmd)

	# visualization
	program = "python " + carmen_path + "/src/segmap/scripts/visualization_and_analysis/generate_graphslam_image.py"
	cmd = "%s %s %s" % (program, output_path, img_path)
	run_command(cmd)


def run_loop_closures(carmen_path, log_path, output_dir, mode):
	global IGNORE_POINTS_ABOVE, IGNORE_POINTS_BELOW, SKIP_WHEN_VELOCITY_IS_BELOW

	program = carmen_path + "/src/segmap/gicp/generate_loop_closures"
	odom_calib = output_dir + "/odom_calib.txt" 
	fused_odom = output_dir + "/fused_odom.txt"
	
	cmd = "%s %s -o %s -f %s" % (program, log_path, odom_calib, fused_odom)
	
	if mode == "gicp":
		gicp_args = " --mode gicp --dist_to_accumulate 2.0 --ignore_above_threshold %lf --ignore_below_threshold %lf --v_thresh %lf" % (IGNORE_POINTS_ABOVE, IGNORE_POINTS_BELOW, SKIP_WHEN_VELOCITY_IS_BELOW)
		gicp_output = " " + output_dir + "/loops.txt"
		run_command(cmd + gicp_output + gicp_args)

	elif mode == "particle_filter":
		pf_args = " --mode particle_filter --n_particles 50 --gps_xy_std 2.0 --gps_h_std 20 --dist_to_accumulate 20.0 --loop_dist 5.0 --n_corrections_when_reinit 20 --v_thresh %lf" % (SKIP_WHEN_VELOCITY_IS_BELOW)
		pf_output = " " + output_dir + "/pf_loops.txt"
		run_command(cmd + pf_output + pf_args)
	elif mode == "localization":
		if 'aeroport' in log_path:
			loop_closure_time = 10
		else:
			loop_closure_time = 60
	
		loc_args = " --mode localization --n_particles 200 --gps_xy_std 2.5 --gps_h_std 20 --dist_to_accumulate 20.0 --loop_dist 10.0 --n_corrections_when_reinit 20 --v_thresh %lf -v 1 --time_dist %lf --v_std 0.5 --phi_std 1.0 --odom_xy_std 0.05 --odom_h_std 0.15 --color_red_std 1 --color_green_std 1 --color_blue_std 1" % (SKIP_WHEN_VELOCITY_IS_BELOW, loop_closure_time)
		loc_output = " " + output_dir + "/localization_loops.txt"
		run_command(cmd + loc_output + loc_args)
	else:
		sys.exit(print("Error: Invalid mode '%s'" % mode))


def main(log_path, skip_until):
	global DATA_DIR

	if not os.path.exists(DATA_DIR):
		os.mkdir(DATA_DIR)

	carmen_path = os.getenv("CARMEN_HOME")	 
	output_dir = create_output_dir(log_path)

	if (not skip_until) or (skip_until <= 1):
		run_odom_calib(carmen_path, log_path, output_dir)
	if (not skip_until) or (skip_until <= 2):
		run_graphslam(carmen_path, log_path, output_dir, "fused_odom")	
#	if (not skip_until) or (skip_until <= 3):
#		run_loop_closures(carmen_path, log_path, output_dir, 'gicp')
#	if (not skip_until) or (skip_until <= 4):
#		run_loop_closures(carmen_path, log_path, output_dir, 'particle_filter')
	if (not skip_until) or (skip_until <= 5):
		run_loop_closures(carmen_path, log_path, output_dir, 'localization')
	if (not skip_until) or (skip_until <= 6):
		run_graphslam(carmen_path, log_path, output_dir, "graphslam")
	if skip_until and skip_until > 6:
		print("Skipped all steps.")

	print("Done.")


if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='Script to optimize poses of a log.')		
	parser.add_argument('log', help='Path to a log.')
	parser.add_argument('--skip', help='Skip all steps of optimization until the chosen one. Possible values: [1. odom_calib | 2. fused_odom | 3. loops with gicp | 4. loops with pf | 5. loops with localization | 6. graphslam]', type=int)
	args = parser.parse_args()
	main(args.log, args.skip)


