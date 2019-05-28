
import os
import sys
from experiments import *


def main(log_path):
    global PARAM_FILE
    output_dir = create_output_dir(log_path)
    poses_path = output_dir + "/graphslam.txt"
    carmen_path = os.getenv("CARMEN_HOME")     

    if not os.path.exists(poses_path):
        print("\nOptimizing poses.\n")
        cmd = "python " + carmen_path + "/src/segmap/scripts/experiments/run_graphslam_for_a_log.py " + log_path
        run_command(cmd)

    map_path = output_dir + "/map"
    
    # assumes we want to re-create the map 
    if os.path.exists(map_path):
        os.system("rm -r " + map_path)
    
    cmd = carmen_path + "/src/segmap/mapper %s %s -m %s --v_thresh %lf" % (log_path, PARAM_FILE, map_path, SKIP_WHEN_VELOCITY_IS_BELOW)
    run_command(cmd)
    
    # visualization
    program = carmen_path + "/src/segmap/scripts/visualization_and_analysis/generate_complete_map.py"
    image_path = output_dir + "/image_map.png"
    cmd = "python %s %s %s" % (program, map_path, image_path)
    run_command(cmd)

    print("Done.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("\nUse python %s <log path>" % sys.argv[0])
        sys.exit(0)
    else:
        main(sys.argv[1])
        
        
        
        
