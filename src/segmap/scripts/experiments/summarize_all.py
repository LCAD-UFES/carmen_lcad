
from experiments import run_command
from multiprocessing import Pool
import os

results_dir = os.getenv("CARMEN_HOME") + "/src/segmap/precious_results2"
results_files = os.listdir(results_dir)

cmds = []
for r in results_files:
	s = r.rsplit("log")
	log_name = "log" + s[1]
	method = s[0]
	log_name = log_name.replace(".txt.txt", ".txt")
	result_file = results_dir + "/" + r
	gt_file = "/dados/data2/data_%s/graphslam_to_map.txt" % log_name
	odometry_file = "precious_odometries/odometries_%s_%s" % (method, log_name)
	summary_file = "experiments_statistics/summary_" + r
	report_file = "experiments_statistics/report_" + r
	program = "scripts/visualization_and_analysis/compute_performance_statistics"
	cmd = "./%s %s %s %s %s %s" % (program, gt_file, result_file, odometry_file, summary_file, report_file)
	cmds.append(cmd)

	# remove invalid lines from the file	
	run_command("sed -i '/^[0-9]* of/!d' %s" % result_file)
	run_command(cmd)

#process_pool = Pool(len(cmds)) 
#process_pool.map(run_command, cmds)
#process_pool.map(print, cmds)
#print("\nDone.")
	

