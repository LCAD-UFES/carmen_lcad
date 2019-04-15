
import os
from multiprocessing import Pool
from experiments import *

USE_LANE_SEGMENTATION = False
	
	
def run_mapper(log_and_type):
	global USE_LANE_SEGMENTATION
	
	log_name = log_and_type[0]
	intensity_mode = log_and_type[1]
	
	mapper_args = " -v 0 "

	if (intensity_mode == 'semantic') and (not os.path.exists('/dados/data/data_%s/semantic' % log_name)):
		print("Semantic segmentations not found for log '%s'. Skipping creation of semantic maps." % log_name)
		return
	
	if (intensity_mode == 'semantic') and (USE_LANE_SEGMENTATION) and ("aeroporto" in log_name):
		mapper_args += " --segment_lane_marks 1 "
	
	if ("aeroporto" in log_name):
		mapper_args += " --camera_latency 0.42 "
	else:
		localization_args += " --camera_latency 0.0 "
		mapper_args += " --camera_latency 0.0 "

	map_path = "/dados/maps2/%s_%s" % (log_name, intensity_mode)
	mapper_cmd = "time ./mapper /dados/%s --v_thresh 1 -i %s -m  %s %s" % (log_name, intensity_mode, map_path, mapper_args)
	run_command(mapper_cmd)

	map_img_path = '/dados/maps2/%s_%s_complete.png' % (log_name, intensity_mode)
	create_complete_map_cmd = 'python scripts/visualization_and_analysis/generate_complete_map.py %s %s' % (map_path, map_img_path)
	run_command(create_complete_map_cmd)
	

if __name__ == "__main__":
	global experiments
	logs_to_map = []
	
	if not os.path.exists('/dados/maps2/'):
		os.mkdir('/dados/maps2')
	
	for e in experiments:
		logs_to_map.append([e['map'], 'semantic'])
		logs_to_map.append([e['map'], 'raw'])
		logs_to_map.append([e['map'], 'visual'])
	
	process_pool = Pool(len(logs_to_map)) 
	process_pool.map(run_mapper, logs_to_map)
	print("Done.")
		
