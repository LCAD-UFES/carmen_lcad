
import os
import numpy as np

d = '/dados/data/'
dirs = [f for f in os.listdir(d) if 'data' == f[:4]]

for f in dirs:
    gps = np.array([l.rstrip().rsplit() for l in open(d + '/' + f + '/optimized.txt', 'r').readlines()])[:, 1:3].astype(float)
    size = np.sum([((gps[i][0] - gps[i-1][0]) ** 2 + (gps[i][1] - gps[i-1][1]) ** 2) ** 0.5 for i in range(1, len(gps))])
    print(f, len(gps), '%.2f km' % (size / 1e3))
