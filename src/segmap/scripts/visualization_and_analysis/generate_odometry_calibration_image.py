
import matplotlib.pyplot as plt 
import numpy as np
import os, sys

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("\nUse python %s <odom_calib_repot.txt> <image_path>")
    else:
        ls = np.array([l.rstrip().rsplit()[1:] for l in open(sys.argv[1], "r").readlines() if "DATA" in l]).astype("float")
        plt.plot(ls[:,0], ls[:,1], linestyle='solid', color='red')
        plt.plot(ls[:,2], ls[:,3], linestyle='solid', color='blue')
        plt.savefig(sys.argv[2])
        print("saved " + sys.argv[2])
