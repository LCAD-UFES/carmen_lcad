#import png
import os
import numpy as np
from math import *
from scipy.misc import toimage
from scipy.ndimage import rotate
from PIL import Image

def getRoi(mat, x, y, roi):
	half = int(round(roi/2))
	roi_x_min = (x - half)
	roi_x_max = (x + half)
	roi_y_min = (y - half)
	roi_y_max = (y + half)
	# Se algum valor ter dimensoes que passam o tamanho da imagem, vai para proximoo arquivo
	if(roi_x_min < 0 or roi_y_min < 0 or roi_x_max >= mat.shape[1] or roi_y_max >= mat.shape[0]):
		return None
	roi_map_dimensions = (int(roi), int(roi))
	roi_map = np.zeros(roi_map_dimensions)
	for k in range(0,roi_map_dimensions[0]):
		for l in range(0, roi_map_dimensions[1]):
			roi_map[k][l] = mat[roi_y_min+k][roi_x_min+l]
	return roi_map

def getCsv(filename, size):
	mat_dimensions = (size, size)
	mat = np.zeros(mat_dimensions)
	f = open(filename, 'r')
	i = 0
	for line in f:
		j = 0
		vals = map(float, line.split(","))
		for val in vals:
			mat[i][j] = val
			j = j + 1
		i = i + 1
	return mat

path = "/dados/neural_mapper_raw_dataset/60mts/"
outpath = "/dados/neural_mapper_png_dataset/60mts/"
radius = 60 #metros
output_side_size = sqrt(2)*radius #metros
#1506400158.422856_label_118.775_87.267_0.20_0.649292
i = 0
for filename in os.listdir(path):
	print(filename)
	#Pega informacoes do nome
	configs = filename.split("_")
	timestamp = configs[0]
	variable = configs[1]
	x_m = float(configs[2])
	y_m = float(configs[3])
	resolution = float(configs[4])
	theta = float(configs[5][:-4])
	# Abre o arquivo e cria matriz com dimensoes da imagem
	mat = getCsv(path+filename, 1050)
	# retira matriz com apenas a regiao de interesse
	x_car_pixel = int(round(x_m/resolution))
	y_car_pixel = int(round(y_m/resolution))
	roi = int(round(2*radius/resolution))
	roi_map = getRoi(mat, x_car_pixel, y_car_pixel, roi)
	if(roi_map is not None):
		# Angulo de rotacao em graus
		tethaDegrees = (theta*180/pi)
		#rotaciona
		cval = -1.0
		if(variable == 'label'):
			cval = 0.0
		roi_map = rotate(roi_map, tethaDegrees, reshape=False, cval=cval, order=1) 
		# Limita para o quadrado inscrito dentro do circulo do raio
		output_size_pixels = int(round(output_side_size/resolution))
		xy = int(round(roi_map.shape[0]/2))
		roi_map = getRoi(roi_map, xy, xy, output_size_pixels)	
		#grava png
		if(variable == 'label'):
			out_name = outpath+ 'labels/' + str(i) + '_view' + '.png'
			toimage(roi_map, cmin=1.0, cmax=3.0).save(out_name)
			roi_map = np.around(roi_map).astype('uint8')
			out_name = outpath+ 'labels/' + str(i) + '_' + variable + '.png'
			toimage(roi_map, low=np.min(roi_map), high=np.max(roi_map)).save(out_name)
			i = i + 1
		else:
			out_name = outpath + 'data/' + str(i) + '_' + variable + '.png'
			toimage(roi_map, cmin=-1.0, cmax=15.0).save(out_name)








