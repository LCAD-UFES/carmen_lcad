import os
import cv2
import glob
import numpy as np
import math
from asyncore import read

# Rotaciona todas as imagens do banco de dados, deixando no formato de nomes apropriado para treinamento
# Nao cria pastas padrao

predict_path = '/mnt/ssd/neural_mapper_train/volta_da_ufes-20190915_augmented/Experimentos/Com_log_softmax/Normalizado/50_epocas/test_results/'
label_path = '/mnt/ssd/neural_mapper_train/volta_da_ufes-20190915_augmented/Experimentos/Com_log_softmax/Normalizado/50_epocas/test_results/'
list_files = '/mnt/ssd/neural_mapper_train/volta_da_ufes-20190915_augmented/Experimentos/Com_log_softmax/Normalizado/50_epocas/teste_results_list.txt'
size = 600

	#0_min
	#if((img1[i,j][1] == 120 and img2[i,j][1] == 1) or (img1[i,j][1] == 0 and img2[i,j][1] == 3) or (img1[i,j][1] == 255 and img2[i,j][1] == 2)):
	#	counter = counter + 1

def new_img(img1, img2, conf):
	counter = 0
	for i in range(img1.shape[0]):
		for j in range(img1.shape[1]):
			val1 = 0
			val2 = 1
			if(img2[i,j][1] == 120):
				val2 = 0
			if(img2[i,j][1] == 0):
				val2 = 2
				
			if(img1[i,j][1] == 0):
				val1 = 2
			elif(img1[i,j][1] == 255):
				val1 = 1
			update_confusion(val1, val2, conf)

def update_confusion(y, x, conf):
	conf[y][x] = conf[y][x] + 1

def getDatasetList(file_name):
    file = open(file_name)
    content = file.read().splitlines()
    # content_list = content.split('\n')
    # for i in content:
    #     print(i)
    return content
   
filenames = getDatasetList(list_files)

n = 0

conf = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0]])

# print(filenames)
for file in filenames:
# 	sufix = file.split('_')[-1]
# 	print(sufix)
	if(True):
# 		print(n)
# 		index = (file.split('_')[-2]).split('/')[-1]
# 		print (label_path + "label" + str(file) + ".png")
		img1 = cv2.imread(predict_path + "img" + str(file) + ".png")
		img2 = cv2.imread(label_path + "label" + str(file) + ".png")
		new_img(img1, img2, conf)
# 		print(conf)
		n = n + 1

print("Matriz de confusao final:")
print(conf)
