#!/bin/python
# O algoritmo ira contar a quantidade de pixels brancos dentre as imagens

import cv2
import os
from collections import Counter 


pathori = "/home/lcad/carmen_lcad/src/rddf_graph/database_ida_a_guarapari/"

for f in  os.listdir(pathori):
    cont = 0
#    print f +"\n"
    img = cv2.imread(pathori+f)
    #pixels = img[6:9,6:9]
    for i in img:
        for j in i:
            if j[0] == 255 and j[1] == 255 and j[2] == 255:
                cont=cont+1
    #        print j


    print f + "  " + str(cont)    

