import os
import cv2
import glob
import numpy as np
import math

# Rotaciona todas as imagens do banco de dados, deixando no formato de nomes apropriado para treinamento
# Nao cria pastas padrao

predict_path = '/home/seidel/carmen_lcad/src/neural_mapper/pytorch_neural_mapper/debug_imgs/circle-volta_da_ufes20160323-acumulado/'
label_path = '/media/seidel/HD ANDRE/PG/neural_mapper_png_dataset/circle_acumulated_volta_da_ufes20160323/labels/'

size = 600

	#0_min

def cmp(img1, img2, radius, last_mean, n):
	counter = 0
	for i in range(img1.shape[0]):
		for j in range(img1.shape[1]):
			y = i-radius
			x = j-radius
			k = math.sqrt((x*x) + (y*y))
			#if(k < radius):
					#print(img1[i,j], img2[i,j])
			if((img1[i,j][1] == 120 and img2[i,j][1] == 1) or (img1[i,j][1] == 0 and img2[i,j][1] == 3) or (img1[i,j][1] == 255 and img2[i,j][1] == 2)):
				counter = counter + 1
				#print ("contou")
	new_count = counter + (last_mean*n*(img1.shape[0]*img1.shape[1]))
	if(last_mean == 0):
		print(counter, ((n+1)*(img1.shape[0]*img1.shape[1])), counter/((n+1)*(img1.shape[0]*img1.shape[1])))
		return float(counter)/float((n+1)*(img1.shape[0]*img1.shape[1]))
	else:
		return float(new_count)/float((n+1)*(img1.shape[0]*img1.shape[1]))

filenames = glob.glob(predict_path  + "*.png")
n = 0
mean = 0
print(filenames)
for file in filenames:
	sufix = file.split('_')[-1]
	#print(sufix)
	if(sufix == "label.png"):
		print(n)
		index = (file.split('_')[-3]).split('/')[-1]
		img1 = cv2.imread(file)
		img2 = cv2.imread(label_path + str(index) + "_label.png")
		radius = size/2
		mean = cmp(img1, img2, radius, mean, n)
		n = n + 1
		if n==100:
			break

print(mean*100)

