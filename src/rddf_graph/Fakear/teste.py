
import os
import cv2
import numpy as np
pathori = "classificadas/"


for f in  os.listdir(pathori):
    print np.matrix(list(f[7:16])).reshape((3, 3)) 

