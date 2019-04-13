#!/bin/python
import os
import cv2


pathori = "classificadas/"
pathmod = "ImagensModificadas/"


#1
for f in  os.listdir(pathori):
    image = cv2.imread(pathori+f)
    image[6:9,6:9] = [0,242,0] 
    image[6,6] = [0,255,0]
    image[6,8] = [0,255,0]
    image[7,7] = [0,255,0]
    cv2.imwrite(pathmod+f[0:-4]+"__1.png", image)
#2
for f in  os.listdir(pathori):
    image = cv2.imread(pathori+f)
    image[6:9,6:9] = [0,242,0]
    image[8,6] = [0,255,0]
    image[8,8] = [0,255,0]
    image[7,7] = [0,255,0]
    cv2.imwrite(pathmod+f[0:-4]+"__2.png", image)
#3
for f in  os.listdir(pathori):
    image = cv2.imread(pathori+f)
    image[6:9,6:9] = [0,242,0]
    image[6,6] = [0,255,0]
    image[8,6] = [0,255,0]
    image[7,7] = [0,255,0]
    cv2.imwrite(pathmod+f[0:-4]+"__3.png", image)
#4
for f in  os.listdir(pathori):
    image = cv2.imread(pathori+f)
    image[6:9,6:9] = [0,242,0]
    image[6,8] = [0,255,0]
    image[8,8] = [0,255,0]
    image[7,7] = [0,255,0]
    cv2.imwrite(pathmod+f[0:-4]+"__4.png", image)
#5
for f in  os.listdir(pathori):
    image = cv2.imread(pathori+f)
    image[6:9,6:9] = [0,242,0]
    image[6,7] = [0,255,0]
    image[8,7] = [0,255,0]
    image[7,7] = [0,255,0]
    cv2.imwrite(pathmod+f[0:-4]+"__5.png", image)
#6
for f in  os.listdir(pathori):
    image = cv2.imread(pathori+f)
    image[6:9,6:9] = [0,242,0]
    image[7,6] = [0,255,0]
    image[7,8] = [0,255,0]
    image[7,7] = [0,255,0]
    cv2.imwrite(pathmod+f[0:-4]+"__6.png", image)
#7
for f in  os.listdir(pathori):
    image = cv2.imread(pathori+f)
    image[6:9,6:9] = [0,242,0]
    image[6,6] = [0,255,0]
    image[8,8] = [0,255,0]
    image[7,7] = [0,255,0]
    cv2.imwrite(pathmod+f[0:-4]+"__7.png", image)
#8
for f in  os.listdir(pathori):
    image = cv2.imread(pathori+f)
    image[6:9,6:9] = [0,242,0]
    image[6,8] = [0,255,0]
    image[8,6] = [0,255,0]
    image[7,7] = [0,255,0]
    cv2.imwrite(pathmod+f[0:-4]+"__8.png", image)

