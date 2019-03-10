
import matplotlib.pyplot as plt 
import numpy as np
import os
import sys


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("\nUse python %s <graphslam poses.txt> <image_path>")
    else:
        ls = np.array([l.rstrip().rsplit() for l in open(sys.argv[1], "r").readlines()]).astype("float")
        plt.plot(ls[:,1], ls[:,2], linestyle='solid', color='red', linewidth=.5)
        plt.plot(ls[:,5], ls[:,6], linestyle='solid', color='blue', linewidth=.5)
        plt.savefig(sys.argv[2], dpi=400)
        print("saved " + sys.argv[2])
