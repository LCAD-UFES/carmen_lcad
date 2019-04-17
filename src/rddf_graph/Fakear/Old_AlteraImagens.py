#!/bin/python
import os
import cv2

pathori = "ImagensOriginais/"
pathmod = "ImagensModificadas/"


for f in  os.listdir(pathori):
    img = cv2.imread(pathori+f)
    img[6,6] = [11, 252, 5]
    img[6,7] = [9, 235, 4]
    img[6,8] = [11, 252, 5]
    img[7,6] = [9, 235, 4]
    img[7,7] = [11, 252, 5]
    img[7,8] = [9, 235, 4]
    img[8,6] = [9, 235, 4]
    img[8,7] = [9, 235, 4]
    img[8,8] = [9, 235, 4]
    cv2.imwrite(pathmod+f[0:-4]+"__1.jpg", img)

for f in  os.listdir(pathori):
    img = cv2.imread(pathori+f)
    img[6,6] = [9, 235, 4]
    img[6,7] = [9, 235, 4]
    img[6,8] = [9, 235, 4]
    img[7,6] = [9, 235, 4]
    img[7,7] = [11, 252, 5]
    img[7,8] = [9, 235, 4]
    img[8,6] = [11, 252, 5]
    img[8,7] = [9, 235, 4]
    img[8,8] = [11, 252, 5]
    cv2.imwrite(pathmod+f[0:-4]+"__2.jpg", img)

for f in  os.listdir(pathori):
    img = cv2.imread(pathori+f)
    img[6,6] = [11, 252, 5]
    img[6,7] = [9, 235, 4]
    img[6,8] = [9, 235, 4]
    img[7,6] = [9, 235, 4]
    img[7,7] = [11, 252, 5]
    img[7,8] = [9, 235, 4]
    img[8,6] = [11, 252, 5]
    img[8,7] = [9, 235, 4]
    img[8,8] = [9, 235, 4]
    cv2.imwrite(pathmod+f[0:-4]+"__3.jpg", img)

for f in  os.listdir(pathori):
    img = cv2.imread(pathori+f)
    img[6,6] = [9, 235, 4]
    img[6,7] = [9, 235, 4]
    img[6,8] = [11, 252, 5]
    img[7,6] = [9, 235, 4]
    img[7,7] = [11, 252, 5]
    img[7,8] = [9, 235, 4]
    img[8,6] = [9, 235, 4]
    img[8,7] = [9, 235, 4]
    img[8,8] = [11, 252, 5]
    cv2.imwrite(pathmod+f[0:-4]+"__4.jpg", img)

for f in  os.listdir(pathori):
    img = cv2.imread(pathori+f)
    img[6,6] = [9, 235, 4]
    img[6,7] = [11, 252, 5]
    img[6,8] = [9, 235, 4]
    img[7,6] = [9, 235, 4]
    img[7,7] = [11, 252, 5]
    img[7,8] = [9, 235, 4]
    img[8,6] = [9, 235, 4]
    img[8,7] = [11, 252, 5]
    img[8,8] = [9, 235, 4]
    cv2.imwrite(pathmod+f[0:-4]+"__5.jpg", img)

for f in  os.listdir(pathori):
    img = cv2.imread(pathori+f)
    img[6,6] = [9, 235, 4]
    img[6,7] = [9, 235, 4]
    img[6,8] = [9, 235, 4]
    img[7,6] = [11, 252, 5]
    img[7,7] = [11, 252, 5]
    img[7,8] = [11, 252, 5]
    img[8,6] = [9, 235, 4]
    img[8,7] = [9, 235, 4]
    img[8,8] = [9, 235, 4]
    cv2.imwrite(pathmod+f[0:-4]+"__6.jpg", img)

for f in  os.listdir(pathori):
    img = cv2.imread(pathori+f)
    img[6,6] = [11, 252, 5]
    img[6,7] = [9, 235, 4]
    img[6,8] = [9, 235, 4]
    img[7,6] = [9, 235, 4]
    img[7,7] = [11, 252, 5]
    img[7,8] = [9, 235, 4]
    img[8,6] = [9, 235, 4]
    img[8,7] = [9, 235, 4]
    img[8,8] = [11, 252, 5]
    cv2.imwrite(pathmod+f[0:-4]+"__7.jpg", img)

for f in  os.listdir(pathori):
    img = cv2.imread(pathori+f)
    img[6,6] = [9, 235, 4]
    img[6,7] = [9, 235, 4]
    img[6,8] = [11, 252, 5]
    img[7,6] = [9, 235, 4]
    img[7,7] = [11, 252, 5]
    img[7,8] = [9, 235, 4]
    img[8,6] = [11, 252, 5]
    img[8,7] = [9, 235, 4]
    img[8,8] = [9, 235, 4]
    cv2.imwrite(pathmod+f[0:-4]+"__8.jpg", img)
