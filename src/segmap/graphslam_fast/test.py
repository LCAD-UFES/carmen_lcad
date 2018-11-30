
import numpy as np
import matplotlib.pyplot as plt

lines = np.array([[float(x) for x in l.rstrip().rsplit()] for l in open('c.txt', 'r').readlines()]) - 1536325964.283733
for l in lines: 
    for x in l:
        print(x - l[0], end=' ')
    print()

