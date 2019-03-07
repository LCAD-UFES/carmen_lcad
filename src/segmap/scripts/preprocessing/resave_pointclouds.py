
import os, glob

def find_logs(path):
    return glob.glob(path + "/log*txt")
    
logs = find_logs("/media/filipe/Hitachi-Teste/")

for log_path in logs:
    if not os.path.exists(log_path):
        raise Exception("Log '%s' not found!" % log_path)

    log_name = log_path.rsplit('/')[-1]
    output_dir = '/dados/data/data_' + log_name + '/velodyne'

    print("log: %s output: %s" % (log_path, output_dir))

    if not os.path.exists(output_dir):
        raise Exception("Output directory '%s' not found!" % output_dir)    

    os.system("grep VELODYNE " + log_path + " > /tmp/velodynes.txt")
    os.system("time ./scripts/convert_pointclouds_to_pcl /tmp/velodynes.txt " + output_dir + " > /tmp/out.txt")



