
import os, sys
from multiprocessing import Pool
from experiments import *

if __name__ == "__main__":
	global experiments
	registration_cmds = []
	graphslam_cmds = []
	
	carmen_path = os.getenv("CARMEN_HOME")	 
	param_file = carmen_path + "/src/carmen-ford-escape.ini"
	
	for e in experiments:
		m = e['map']
		for t in e['test']:
		
			cmd = "time ./gicp/generate_loop_closures_between_logs /dados/%s /dados/%s %s /dados/data2/data_%s/pf_loops_to_map.txt --mode localization --n_particles 200 --gps_xy_std 5.0 --gps_h_std 20 --dist_to_accumulate 20.0 --loop_dist 5.0 --n_corrections_when_reinit 20 --v_thresh 1 -v 1 --time_dist 0 --color_red_std 3 --color_green_std 3 --color_blue_std 3 --reflectivity_std 3 --use_map_weight 1 --clean_map 0 --view_imgs 0 --view_pointcloud 0 " % (m, t, param_file, t)
			registration_cmds.append(cmd + " > /dev/null 2>&1")

			#print(cmd)
			#print()			
			
			if ("brt" in t):
				args = "--gps_xy_std 50.00000 --gps_angle_std 50.000000 --pf_loops_xy_std 0.1 --pf_loops_angle_std 0.1  --gps_step 50"
			elif ("jardim" in t):
				args = "--gps_xy_std 5.00000 --gps_angle_std 20.000000 --pf_loops_xy_std 0.05 --pf_loops_angle_std 1.0 "
			else:
				args = "--gps_xy_std 2.00000 --gps_angle_std 20.000000 --pf_loops_xy_std 0.05 --pf_loops_angle_std 1.0 "

			cmd = "/home/filipe/carmen_lcad/src/segmap/graphslam_fast/graphslam /dados/%s ../carmen-ford-escape.ini /dados/data2//data_%s//graphslam_to_map.txt -o /dados/data2//data_%s//odom_calib.txt -l /dados/data2//data_%s//loops.txt --pf_loops /dados/data2//data_%s//localization_loops.txt --pf_to_map /dados/data2/data_%s/pf_loops_to_map.txt %s --pf_to_map_xy_std 0.2 --pf_to_map_angle_std 2 " % (t, t, t, t, t, t, args)
			
			graphslam_cmds.append(cmd)

	print("Running processes.")
	#process_pool = Pool(len(registration_cmds))
	#process_pool.map(run_command, registration_cmds)
	process_pool = Pool(len(graphslam_cmds))
	process_pool.map(run_command, graphslam_cmds)
	print("Done.")

		
