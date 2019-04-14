
import os, sys
from multiprocessing import Pool
from experiments import *

if __name__ == "__main__":
    global experiments
    registration_cmds = []
    graphslam_cmds = []
    for e in experiments:
        m = e['map']
        for t in e['test']:
            cmd = "time ./gicp/generate_loop_closures_between_logs /dados/%s /dados/%s /dados/data2/data_%s/pf_loops_to_map.txt --mode localization --n_particles 100 --gps_xy_std 2.5 --gps_h_std 20 --dist_to_accumulate 20.0 --loop_dist 10.0 --n_corrections_when_reinit 20 --v_thresh 1 -v 1 --time_dist 0 --v_std 0.5 --phi_std 1.0 --odom_xy_std 0.05 --odom_h_std 0.15 --color_red_std 1 --color_green_std 1 --color_blue_std 1" % (m, t, t)
            registration_cmds.append(cmd)

            print(cmd)
            print()            
            
            cmd = "/home/filipe/carmen_lcad/src/segmap/graphslam_fast/graphslam /dados/%s /dados/data2//data_%s//graphslam_to_map.txt -o /dados/data2//data_%s//odom_calib.txt -l /dados/data2//data_%s//loops.txt --pf_loops /dados/data2//data_%s//localization_loops.txt --pf_to_map /dados/data2/data_%s/pf_loops_to_map.txt --gps_xy_std 10.000000 --gps_angle_std 10.000000 --gicp_loops_xy_std 0.300000 --gicp_loops_angle_std 1.000000 --pf_loops_xy_std 0.02 --pf_loops_angle_std 3 --pf_to_map_xy_std 0.1 --pf_to_map_angle_std 10" % (t, t, t, t, t, t)
            
            print(cmd)
            print()
            
            graphslam_cmds.append(cmd)

    print("Running processes.")
    run_command("rm -rf /tmp/map*")
    process_pool = Pool(len(registration_cmds))
    process_pool.map(run_command, registration_cmds)
    process_pool = Pool(len(graphslam_cmds))
    process_pool.map(run_command, graphslam_cmds)
    print("Done.")

        
