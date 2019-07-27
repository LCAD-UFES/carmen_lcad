

import os
from multiprocessing import Pool
from experiments import *


def run_mapper(log_name, intensity_mode, map_type, use_lane_segmentation):
	mapper_args = " -v 1 " 

	if (intensity_mode == 'semantic') and (not os.path.exists('/dados/data/data_%s/semantic' % log_name)):
		print("Semantic segmentations not found for log '%s'. Skipping creation of semantic maps." % log_name)
		return
	
	# we only want to create map using lane segmentation in airport logs
	if ((use_lane_segmentation) and ("aeroporto" not in log_name)):
		return
	
	map_path = "/dados/maps2/%s_%s" % (log_name, intensity_mode)

	tag = ""

	if (intensity_mode == 'semantic') and (use_lane_segmentation) and ("aeroporto" in log_name):
		mapper_args += " --segment_lane_marks 1 "
		tag = "_with_lanes"
	
	if 'aeroport' in log_name or 'noite' in log_name:
		mapper_args += " --camera_latency 0.3 "
	else:
		mapper_args += " --camera_latency 0.0 "

	map_path += tag
	run_command("rm -rf %s/*" % map_path)

	mapper_cmd = "time ./mapper /dados/%s ../carmen-ford-escape.ini --v_thresh 1 -i %s -m  %s %s" % (log_name, intensity_mode, map_path, mapper_args)
	run_command(mapper_cmd)

	map_img_path = map_path + "_complete.png"
	create_complete_map_cmd = 'python scripts/visualization_and_analysis/generate_complete_map.py %s %s' % (map_path, map_img_path)
	run_command(create_complete_map_cmd)
	

if __name__ == "__main__":
	global experiments
	logs_to_map = []
	
	if not os.path.exists('/dados/maps2/'):
		os.mkdir('/dados/maps2')
	
	for e in experiments:
		logs_to_map.append([e['map'], 'semantic', 'semantic', True])
		logs_to_map.append([e['map'], 'semantic', 'semantic', False])
		logs_to_map.append([e['map'], 'reflectivity', False])
		logs_to_map.append([e['map'], 'colour', 'colour', False])
		logs_to_map.append([e['map'], 'reflectivity', 'occupancy', False])
	
	process_pool = Pool(len(logs_to_map)) 
	process_pool.map(run_mapper, logs_to_map)
	print("Done.")
		
