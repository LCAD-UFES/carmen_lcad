
import os
import sys
import time
from multiprocessing import Process

def run(cmd):
    os.system(cmd)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("\nUse python {} <cmds file> <max_capacity>".format(sys.argv[0]))
    else:
        cmds = open(sys.argv[1]).readlines()
        max_capacity = int(sys.argv[2])

        running_procs = []
        all_procs = []
        already_closed = []

        for i in range(len(cmds)):
            while len(running_procs) >= max_capacity:
                time.sleep(2)
                running_procs = []
                for j in range(len(all_procs)):
                    p = all_procs[j]
                    if not p.is_alive():
                        if not already_closed[j]:
                            # join is necessary to release the resources once the process ends.
                            p.join()
                            already_closed[j] = True
                    else:
                        running_procs.append(p)

            p = Process(target=run, args=(cmds[i], ))
            p.start()

            print('Running process', p.pid, i+1, 'of', len(cmds),
                  'Percentage: %.1f' % (100.0 * (float(i+1) / len(cmds))), 'cmd:', cmds[i])

            all_procs.append(p)
            running_procs.append(p)
            already_closed.append(False)
            time.sleep(2)

        for i in range(len(all_procs)):
            p = all_procs[i]
            if not p.is_alive():
                if not already_closed[i]:
                    # join is necessary to release the resources once the process ends.
                    p.join()
                    already_closed[i] = True
