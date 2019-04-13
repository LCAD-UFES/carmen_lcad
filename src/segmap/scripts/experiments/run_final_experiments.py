
import os
from multiprocessing import Pool
from experiments import *

def run_experiment(pair):
    m = pair[0]
    t = pair[1]
    mapper_cmd = "time ./mapper /dados/%s --v_thresh 1 -m /tmp/semantic_map_%s -i semantic" % (m, m)
    localization_args = "--n_particles 200 --gps_xy_std 2.5 --gps_h_std 20 --color_red_std 1 --color_green_std 1 --color_blue_std 1"
    localization_cmd = "time ./localizer /dados/%s -m /tmp/semantic_map_%s -i semantic %s > localization_result_%s.txt" % (t, m, localization_args, t)

    run_command(mapper_cmd)
    run_command(localization_cmd)    


if __name__ == "__main__":
    global experiments
    pairs = []
    for e in experiments:
        for t in e['test']:
            pairs.append([e['map'], t])
    
    process_pool = Pool(len(pairs)) 
    process_pool.map(run_experiment, pairs)
    print("Done.")
        
