
import os, sys
from multiprocessing import Pool
from experiments import *

if __name__ == "__main__":
    global experiments
    all_logs = []
    for e in experiments:
        all_logs += [e['map']]
        all_logs += e['test']

    cmds = []
    for l in all_logs:
        cmds.append('python scripts/experiments/run_graphslam_for_a_log.py /dados/' + l)

    print("Running processes.")
    run_command("rm -rf /tmp/map*")
    process_pool = Pool(len(cmds))
    process_pool.map(run_command, cmds)
    print("Done.")

