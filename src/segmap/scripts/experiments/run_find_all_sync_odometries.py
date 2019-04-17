
from multiprocessing import Pool
from experiments import run_command
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
	
	program = "scripts/visualization_and_analysis/find_synchronized_odometry"
	output_path = "precious_odometries/odometries_%s_%s" % (method, log_name)
	
	cmd = "./%s /dados/%s %s %s" % (program, log_name, result_file, output_path)

	# remove invalid lines from the file	
	run_command("sed -i '/^[0-9]* of/!d' %s" % result_file)
	cmds.append(cmd)
	
process_pool = Pool(len(cmds)) 
process_pool.map(run_command, cmds)

