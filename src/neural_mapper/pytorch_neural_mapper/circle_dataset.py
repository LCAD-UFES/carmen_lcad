#Hot to use:
# Crie a pasta de saida no mesmo padrao da do data set
# -Dataset
#  |--data
#  |--labels
# Coloque no size o tamanho da area

import os
import cv2
import glob
import numpy as np
import math

# Rotaciona todas as imagens do banco de dados, deixando no formato de nomes apropriado para treinamento
# Nao cria pastas padrao
in_path = '/media/vinicius/NewHD/Datasets/Neural_Mapper_dataset/60mts_guarapari'
out_path = '/media/vinicius/NewHD/Datasets/Neural_Mapper_dataset/60mts_guarapari_circ'

data_in_path = in_path + '/data/'
label_in_path = in_path + '/labels/'

size = 600

	#0_min

def circle(img, radius, is_label):
	out_img = img
	for i in range(img.shape[0]):
		for j in range(img.shape[1]):
			y = i-radius
			x = j-radius
			k = math.sqrt((x*x) + (y*y))
			if(k > radius):
				if(is_label):
					out_img[i,j] = 1
				else:
					out_img[i,j] = 0
	return out_img

filenames = glob.glob(data_in_path + '*.png')
for file in glob.glob(label_in_path + '*.png'):
	filenames.append(file)

for file in filenames:
	index = (file.split('_')[-3]).split('/')[-1]
	sufix = file.split('_')[-1]
	data_dir = file.split('/')[-2]
	is_label = (sufix == "label.png")
	print(is_label)
	in_img = cv2.imread(file)
	radius = size/2
	out_img = circle(in_img, radius, is_label)
	#print(out_path, data_dir, index, sufix)
	out_name = out_path + '/' + data_dir + '/' + str(index) + '_' + sufix
	print (out_name)
	cv2.imwrite(out_name, out_img)

