
import os
from multiprocessing import Pool
from experiments import *

def run_experiment(pair):
	m = pair[0]
	t = pair[1]
	use_lane_segmentation = pair[2]
	mode = pair[3]
	
	localization_args = "--n_particles 200 --gps_xy_std 2.5 --gps_h_std 20 --color_red_std 1 --color_green_std 1 --color_blue_std 1"
	
	map_path = "/dados/maps2/%s_%s" % (m, mode)
	
	if (use_lane_segmentation and "aeroporto" not in t):
		return
	
	if use_lane_segmentation and ("aeroporto" in t or "aeroporto" in m):
		map_path += "_with_lanes"
		localization_args += " --segment_lane_marks 1 "
		output = "localization_%s_with_lane_%s.txt" % (mode, t)
	else:
		output = "localization_%s_%s.txt" % (mode, t)
	
	if ("aeroporto" in t or "aeroporto" in m):
		localization_args += " --camera_latency 0.42 "
	else:
		localization_args += " --camera_latency 0.0 "
	
	localization_cmd = "time ./localizer /dados/%s ../carmen-ford-escape.ini -m %s -i %s %s --save_dir imgs_localizer_%s > %s 2> /dev/null" % (t, map_path, mode, localization_args, t, output)
	run_command(localization_cmd)	


if __name__ == "__main__":
	global experiments
	pairs = []
	for e in experiments:
		for t in e['test']:
			pairs.append([e['map'], t, False, "remission"])
			pairs.append([e['map'], t, False, "visual"])			
			pairs.append([e['map'], t, False, "semantic"])												
			pairs.append([e['map'], t, True, "semantic"])
	
	process_pool = Pool(len(pairs)) 
	process_pool.map(run_experiment, pairs)
	print("Done.")
		
