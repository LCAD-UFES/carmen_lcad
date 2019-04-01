#!/bin/python

import cv2

def soma(elem):
    return sum(elem)/3

image = cv2.imread("image8.png")

pixel= image[7, 7]
#print image
lista = []
x = image[6:9,6:9]
for i in x:
    for y in i:
        lista.append(y)
        #print soma(y)
        #print y

lista.sort(key=soma, reverse=True)


with open('temp.txt', 'w') as f:
    for item in lista:
        f.write(str(item)+'\n')



image[6,6] = lista[7]
image[6,7] = lista[5]
image[6,8] = lista[6]
image[7,6] = lista[4]
image[7,7] = lista[8]
image[7,8] = lista[3]
image[8,6] = lista[2]
image[8,7] = lista[1]
image[8,8] = lista[0]




#cv2.imwrite("testando2.png", image) 
