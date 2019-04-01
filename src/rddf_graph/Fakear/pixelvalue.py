#!/bin/python
# No momento ele esta contando os pixels 255 mais comuns de todas as imagens classificadas

import cv2
import os
from collections import Counter 

#image = cv2.imread("testando2.png")

pathori = "classificadas/"

list1=[]

for f in  os.listdir(pathori):
#    print f +"\n"
    temp = ""
    img = cv2.imread(pathori+f)
    pixels = img[6:9,6:9]
    for i in pixels:
        for j in i:
            if j[0] != 0 or j[1] != 255 or j[2] != 0:
                j = [0,0,0]
                temp=temp+"0"
            else:
                temp=temp+"1"
#    print "\n"
    list1.append(temp)





    
print Counter(list1) 
