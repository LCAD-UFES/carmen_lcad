
import time
from multiprocessing import Pool
import os

d = "/media/filipe/Hitachi-Teste/"

def process_log(l):
    log_path = d + '/' + l
    raw_sync_path = "/tmp/raw_sync_" + l
    sync = "/dados/data/data_" + l + "/sync.txt"
    cmd = "python scripts/step0_parse_and_sync.py %s > %s" % (log_path, raw_sync_path)
    os.system(cmd)
    cmd = "./scripts/convert_latlon_to_xyz %s %s" % (raw_sync_path, sync)
    os.system(cmd)

init = time.time()
logs = [l for l in os.listdir(d) if l[:3] == "log" and l[-4:] == ".txt"]
p = Pool(len(logs))
p.map(process_log, logs)
print("Done in %lf seconds." % (time.time() - init))    

