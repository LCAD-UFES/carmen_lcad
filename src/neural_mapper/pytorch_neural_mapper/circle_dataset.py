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

# Limita ground thruth ao raio do laser de todas as imagens passadas pela lista, deixando no formato de nomes apropriado para treinamento
# Nao cria pastas padrao
in_path = '/dados/neural_mapper/data_13-08-19/data'
out_path = '/dados/neural_mapper/data_13-08-19/ciculado'
img_list = '/dados/neural_mapper/data_13-08-19/data/' 

data_in_path = in_path + '/data/'
label_in_path = in_path + '/labels/'

size = 600

def circle(img, radius):
	out_img = img
	for i in range(img.shape[0]):
		for j in range(img.shape[1]):
			y = i-radius
			x = j-radius
			k = math.sqrt((x*x) + (y*y))
			if(k > radius):
				out_img[i,j] = 0
	return out_img

def getDatasetList(file_name):
    file = open(file_name)
    content = file.read().splitlines()
    return content
   
def file_to_numpy(img_x_dim, img_y_dim, file):
    numpy_file = np.fromfile(file, dtype=float)
    reshaped = numpy_file.reshape(img_x_dim, img_y_dim)
    return reshaped

filenames = getDatasetList(file_name)

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

