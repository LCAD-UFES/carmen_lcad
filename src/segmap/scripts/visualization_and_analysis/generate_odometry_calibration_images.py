
import matplotlib.pyplot as plt 
import numpy as np
import os

base = "/dados/data2"
dirs = os.listdir(base)
dirs = ['data_log_sao_paulo_brt_20170827-2.txt',
        ]

for d in dirs:
    dr = base + "/" + d
    fn = dr + "/report_odom_calib.txt"
    fg = dr + "/report_odom_calib.png"
    ls = np.array([l.rstrip().rsplit()[1:] for l in open(fn, "r").readlines() if "DATA" in l]).astype("float")
    plt.plot(ls[:,0], ls[:,1], linestyle='solid', color='red')
    plt.plot(ls[:,2], ls[:,3], linestyle='solid', color='blue')
    plt.savefig(fg)
    print("saved " + fg)
    plt.clf()

