
import os
from multiprocessing import Pool
from experiments import *

def run_experiment(pair):
	m = pair[0]
	t = pair[1]
	use_lane_segmentation = pair[2]
	
	localization_args = "--n_particles 200 --gps_xy_std 2.5 --gps_h_std 20 --color_red_std 1 --color_green_std 1 --color_blue_std 1"
	
	map_path = "/dados/maps2/%s_semantic" % m
	
	if (use_lane_segmentation and "aeroporto" not in t):
		return
	
	if use_lane_segmentation and ("aeroporto" in t or "aeroporto" in m):
		map_path += "_with_lanes"
		localization_args += " --segment_lane_marks 1 "
		output = "localization_semantic_with_lane_%s.txt" % (t)
	else:
		output = "localization_semantic_%s.txt" % (t)
	
	if ("aeroporto" in t or "aeroporto" in m):
		localization_args += " --camera_latency 0.42 "
	else:
		localization_args += " --camera_latency 0.0 "
	
	localization_cmd = "time ./localizer /dados/%s -m %s -i semantic %s > %s 2> /dev/null" % (t, map_path, localization_args, output)
	run_command(localization_cmd)	


if __name__ == "__main__":
	global experiments
	pairs = []
	for e in experiments:
		for t in e['test']:
			pairs.append([e['map'], t, True])
			pairs.append([e['map'], t, False])			
	
	process_pool = Pool(len(pairs)) 
	process_pool.map(run_experiment, pairs)
	print("Done.")
		
