#How to use:
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
from numpy import std, dtype, float64
import time
#import threading
import multiprocessing
import scipy.sparse
from PIL import Image

'''
 Limita ground thruth ao raio do laser de todas as imagens passadas pela lista, deixando no formato de nomes apropriado para treinamento
Nao cria pastas padrao all_images, precisa apenas do nome da imagem 
ex: a imagem 0_360000_0.000000_7757679.296649_-363600.985947_1491276861.247797 possui na pasta data as 5 estatisticas para ela com o
sufixo _max, _mean, _min, _numb e std (serao carregadas automaticamente) e na pasta label _label é carregado automaticamente também

Para gerar a lista com all_images faca:
cd <dataset_path/labels>
ls -v *_label > ../all_images.txt
abra o arquivo com o gedit ou qualquer outra forma e apague os _label do nome dos arquivos
usei procurar e substituir
'''

dataset_list_file = "/media/vinicius/HD_EXT_VINI/neural_mapper_raw/guarapari-20170403-2_no_bumblebee-part2/all_images.txt"
dataset_path = "/media/vinicius/HD_EXT_VINI/neural_mapper_raw/guarapari-20170403-2_no_bumblebee-part2/"
data_path = "data/"
label_path = "labels/"

out_path = "/mnt/ssd/neural_mapper_train/guarapari-20170403-2_augmented-part2/"

max_threads = 8

#statistics + label + view
input_dimensions = 6

new_count = 0
size = 600
center = 600/2
velodyne_max_range = 60
map_resolution = 0.2
radius = (velodyne_max_range/map_resolution)
new_size = 600
rotate_dataset = True
num_rotations = 4 #codigo ainda nao aceita nada diferente disso

calculate_transforms = False
save_png_to_debug = False

def getDatasetList(file_name):
	file = open(file_name)
	content = file.read().splitlines()
	return content


def file_to_numpy(img_x_dim, img_y_dim, file):
	numpy_file = np.fromfile(file, dtype=float)
	reshaped = numpy_file.reshape(img_x_dim, img_y_dim)
	return reshaped


def load_statistics_label_view(item):
	data_files = dataset_path + data_path + str(item)
	labels_files = dataset_path + label_path + str(item) 

	data = np.zeros((input_dimensions, size, size))
# 	print(data_files)
	data[0] = (file_to_numpy(size,size, data_files + '_max'))
	data[1] = (file_to_numpy(size,size, data_files + '_mean'))
	data[2] = (file_to_numpy(size,size, data_files + '_min'))
	data[3] = (file_to_numpy(size,size, data_files + '_numb'))
	data[4] = (file_to_numpy(size,size, data_files + '_std'))
	
	data[5] = (file_to_numpy(size,size, labels_files + '_label'))
	
# 	for i in data:
# 		print(i)
# 	exit()
	return data


def fixed_normalize_cell(value_map, new_max, last_max, min):
	new_val = value_map
	if(new_val < min):
		new_val = min
	normalized = (new_val-min)*(new_max)/(last_max-min)
	#printf("max = %lf | min = %lf | norm_max = %d | norm_min = %d\n", max, min, normalized[max_index], normalized[min_index]);
	return normalized


def circle_and_mean(data, radius, new_size):
	
	map_min = -1.0
	new_data = np.zeros((input_dimensions, new_size, new_size))
	 
	for i in range(size):
		for j in range(size):
			y = i-center
			x = j-center
			k = math.sqrt((x*x) + (y*y))
			if(k < radius): #new_data already starts with 0
				l = round (i - (center - radius)) 
				m = round (j - (center - radius))
				new_data[0][l][m] = fixed_normalize_cell(data[0][i][j], 1.0, 1.852193, map_min) #max
				new_data[1][l][m] = fixed_normalize_cell(data[1][i][j], 1.0, 1.852193, map_min) #mean
				new_data[2][l][m] = fixed_normalize_cell(data[2][i][j], 1.0, 1.852193, map_min) #min
				new_data[3][l][m] = fixed_normalize_cell(data[3][i][j], 1.0, 64, 	   map_min) #numb
				new_data[4][l][m] = fixed_normalize_cell(data[4][i][j], 1.0, 15,	   map_min) #std
				#label      l  m
				new_data[5][l][m] = data[5][i][j]

	return new_data   


def labels_to_img(numpy_image, size):
	reshaped = numpy_image
	img = np.zeros((size, size, 3), np.uint8)
	for i in range(size):
		# print("")
		for j in range(size):
			if reshaped[i][j] == 0.0:
				img[i][j] = np.array([255, 120, 0])
			elif reshaped[i][j] == 1.0:
				img[i][j] = np.array([255, 255, 255])
			elif reshaped[i][j] == 2.0:
				img[i][j] = np.array([0, 0, 0])
	return img


def calc_variance(data, size, total_mean):
	sum_var_max  = 0.0
	sum_var_mean = 0.0
	sum_var_min  = 0.0
	sum_var_numb = 0.0
	sum_var_std  = 0.0
	
	for i in range(size):
		for j in range(size):
			y = i-radius
			x = j-radius
			k = math.sqrt((x*x) + (y*y))
			if(k < radius): #new_data already starts with 0
				_max  = data[0][i][j] - total_mean[0]#mean_max
				_mean = data[1][i][j] - total_mean[1]#mean_mean
				_min  = data[2][i][j] - total_mean[2]#mean_min 
				_numb = data[3][i][j] - total_mean[3]#mean_numb
				_std  = data[4][i][j] - total_mean[4]#mean_std 
				
				sum_var_max += (_max  * _max )
				sum_var_mean += (_mean * _mean)
				sum_var_min  += (_min  * _min )
				sum_var_numb += (_numb * _numb)
				sum_var_std  += (_std  * _std )

	return sum_var_max, sum_var_mean, sum_var_min, sum_var_numb, sum_var_std   

	
def rotate(new_data, num_rotations):
	rots_data = np.zeros((num_rotations, input_dimensions, size, size))
# 	print(new_data.shape[0])
	
	for i in range(new_data.shape[0]):
		rots_data[0][i] = np.rot90(new_data[i])
		rots_data[1][i] = np.rot90(new_data[i], 2)
		rots_data[2][i] = np.rot90(new_data[i], 3)
		rots_data[3][i] = np.fliplr(new_data[i])
		
	return rots_data
	
def save_rotated(rots_data, rot_id, rotacao, new_count):
	#Dados_rotacionado rotaca graus
	rots_data[rot_id][0].tofile(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_max') 
	rots_data[rot_id][1].tofile(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_mean') 
	rots_data[rot_id][2].tofile(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_min') 
	rots_data[rot_id][3].tofile(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_numb') 
	rots_data[rot_id][4].tofile(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_std') 
	rots_data[rot_id][5].tofile(out_path + label_path + str(new_count) + '_' + str(rotacao) + '_label')
	img = labels_to_img(rots_data[rot_id][5], new_size)
	cv2.imwrite(out_path + label_path + str(new_count) + '_' + str(rotacao) + '_view.png', img)


def save_rotated_sparse(rots_data, rot_id, rotacao, new_count, new_size, name_data):
		
	#Dados_rotacionado rotaca graus
	
	scipy.sparse.save_npz((out_path + data_path + new_count + '_' + str(rotacao) + '_' + name_data + '_max')     ,   scipy.sparse.csr_matrix(rots_data[rot_id][0]))
	scipy.sparse.save_npz((out_path + data_path + new_count + '_' + str(rotacao) + '_' + name_data + '_mean')    ,   scipy.sparse.csr_matrix(rots_data[rot_id][1]))
	scipy.sparse.save_npz((out_path + data_path + new_count + '_' + str(rotacao) + '_' + name_data + '_min')    ,   scipy.sparse.csr_matrix(rots_data[rot_id][2]))
	scipy.sparse.save_npz((out_path + data_path + new_count + '_' + str(rotacao) + '_' + name_data + '_numb')    ,   scipy.sparse.csr_matrix(rots_data[rot_id][3]))
	scipy.sparse.save_npz((out_path + data_path + new_count + '_' + str(rotacao) + '_' + name_data + '_std')    ,   scipy.sparse.csr_matrix(rots_data[rot_id][4]))
	scipy.sparse.save_npz((out_path + label_path + new_count + '_' + str(rotacao) + '_' + name_data + '_label')  ,   scipy.sparse.csr_matrix(rots_data[rot_id][5]))
			
	img = labels_to_img(rots_data[rot_id][5], new_size)
	cv2.imwrite(out_path + label_path + str(new_count) + '_' + str(rotacao) + '_' + name_data + '_view.png', img)

	
def teste_visualize(new_data, new_count, rotacao, name_data):
	
	cv2.imwrite(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_' + name_data + '_max' + '_view.png', (new_data[0]*127))
	cv2.imwrite(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_' + name_data + '_mean' + '_view.png', (new_data[1]*127))
	cv2.imwrite(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_' + name_data + '_min' + '_view.png', (new_data[2]*127))
	cv2.imwrite(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_' + name_data + '_numb' + '_view.png', (new_data[3]*127))
	cv2.imwrite(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_' + name_data + '_std' + '_view.png', (new_data[4]*127))

def teste_visualize_with_rotation(new_count, rots_data, rotacao, rot_id, name_data):
		
	cv2.imwrite(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_' + name_data + '_max' + '_view.png', (rots_data[rot_id][0]*255))
	cv2.imwrite(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_' + name_data + '_mean' + '_view.png', (rots_data[rot_id][1]*255))
	cv2.imwrite(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_' + name_data + '_min' + '_view.png', (rots_data[rot_id][2]*255))
	cv2.imwrite(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_' + name_data + '_numb' + '_view.png', (rots_data[rot_id][3]*255))
	cv2.imwrite(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_' + name_data + '_std' + '_view.png', (rots_data[rot_id][4]*255))
	
		
def save(new_data, rots_data, new_count):
	
	#Dados_circulo
	rotacao = 0
	if save_png_to_debug:
		teste_visualize(new_data, new_count, rotacao)
		
	new_data[0].tofile(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_max')
	new_data[1].tofile(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_mean')
	new_data[2].tofile(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_min')
	new_data[3].tofile(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_numb')
	new_data[4].tofile(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_std')
	new_data[5].tofile(out_path + label_path + str(new_count) + '_' + str(rotacao) + '_label')
	img = labels_to_img(new_data[5], new_size)
	cv2.imwrite(out_path + label_path + str(new_count) + '_' + str(rotacao) + '_view.png', img)
	new_count += 1
	
	if rotate_dataset:
        # 	teste_visualize_with_rotation(new_count, rots_data, 90, 0)
		save_rotated(rots_data, 0, 90, new_count)
		new_count += 1
		save_rotated(rots_data, 1, 180, new_count)
		new_count += 1
		save_rotated(rots_data, 2, 270, new_count)
		new_count += 1

            #Dados_rotacionado flip horizontal graus
		rotacao = 'flipH'
		rots_data[3][0].tofile(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_max') 
		rots_data[3][1].tofile(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_mean') 
		rots_data[3][2].tofile(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_min') 
		rots_data[3][3].tofile(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_numb') 
		rots_data[3][4].tofile(out_path + data_path + str(new_count) + '_' + str(rotacao) + '_std') 
		rots_data[3][5].tofile(out_path + label_path + str(new_count) + '_' +str(rotacao) + '_label')
		img = labels_to_img(rots_data[3][5], new_size)
		cv2.imwrite(out_path + label_path + str(new_count) + '_' + str(rotacao) + '_view.png', img)
		new_count += 1
		
	return new_count


def save_as_sparse_matrix(new_data, rots_data, new_size, item):
	#Dados_circulo
	temp_name = item.split("_")
	name_data = (temp_name[2] + "_" + temp_name[3] + "_" + temp_name[4] + "_" + temp_name[5])
	new_count = temp_name[0]
	rotacao = 0
	if save_png_to_debug:
		teste_visualize(new_data, new_count, rotacao)
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.sparse.save_npz.html
	scipy.sparse.save_npz((out_path + data_path + new_count + '_' + str(rotacao) + '_' + name_data + '_max'),   scipy.sparse.csr_matrix(new_data[0]))
	scipy.sparse.save_npz((out_path + data_path + new_count + '_' + str(rotacao) + '_' + name_data + '_mean')  , scipy.sparse.csr_matrix(new_data[1]))
	scipy.sparse.save_npz((out_path + data_path + new_count + '_' + str(rotacao) + '_' + name_data + '_min')  , scipy.sparse.csr_matrix(new_data[2]))
	scipy.sparse.save_npz((out_path + data_path + new_count + '_' + str(rotacao) + '_' + name_data + '_numb')  , scipy.sparse.csr_matrix(new_data[3]))
	scipy.sparse.save_npz((out_path + data_path + new_count + '_' + str(rotacao) + '_' + name_data + '_std')  , scipy.sparse.csr_matrix(new_data[4]))
	scipy.sparse.save_npz((out_path + label_path + new_count + '_' + str(rotacao) + '_' + name_data + '_label'), scipy.sparse.csr_matrix(new_data[5]))
	img = labels_to_img(new_data[5], new_size)
	cv2.imwrite(out_path + label_path + new_count + '_' + str(rotacao) + '_' + name_data + '_view.png', img)
		
	if rotate_dataset:
        # 	teste_visualize_with_rotation(new_count, rots_data, 90, 0)
		save_rotated_sparse(rots_data, 0, 90, new_count, new_size, name_data)
		save_rotated_sparse(rots_data, 1, 180, new_count, new_size, name_data)
		save_rotated_sparse(rots_data, 2, 270, new_count, new_size, name_data)

            #Dados_rotacionado flip horizontal graus
		rotacao = 'flipH'
		scipy.sparse.save_npz((out_path + data_path + new_count + '_' + str(rotacao) + '_' + name_data + '_max')  ,   scipy.sparse.csr_matrix(rots_data[3][0]))
		scipy.sparse.save_npz((out_path + data_path + new_count + '_' + str(rotacao) + '_' + name_data + '_mean') ,   scipy.sparse.csr_matrix(rots_data[3][1]))
		scipy.sparse.save_npz((out_path + data_path + new_count + '_' + str(rotacao) + '_' + name_data + '_min') ,   scipy.sparse.csr_matrix(rots_data[3][2]))
		scipy.sparse.save_npz((out_path + data_path + new_count + '_' + str(rotacao) + '_' + name_data + '_numb') ,   scipy.sparse.csr_matrix(rots_data[3][3]))
		scipy.sparse.save_npz((out_path + data_path + new_count + '_' + str(rotacao) + '_' + name_data + '_std') ,   scipy.sparse.csr_matrix(rots_data[3][4]))
		scipy.sparse.save_npz((out_path + label_path + new_count + '_' +str(rotacao) + '_' + name_data + '_label'),   scipy.sparse.csr_matrix(rots_data[3][5]))
		
		img = labels_to_img(rots_data[3][5], new_size)
		cv2.imwrite(out_path + label_path + new_count + '_' + str(rotacao) + '_' + name_data + '_view.png', img)
		
		return new_count


def save_as_tiff(new_data, rots_data, new_count):
	
	#Dados_circulo
	rotacao = 0
	#teste_visualize(new_data, new_count, rotacao)
	im = Image.fromarray(new_data[0], mode='F') # float32
	im.save((out_path + data_path + str(new_count) + '_' + str(rotacao) + '_max.tiff'), "TIFF")
	
	im = Image.fromarray(new_data[5], mode='F') # float32
	im.save((out_path + label_path + str(new_count) + '_' + str(rotacao) + '_label.tiff'), "TIFF")
	
	
	return new_count


def calculate_total_mean(mean_max, mean_mean, mean_min, mean_numb, mean_std, num_files):
	
	arq = open(out_path + 'dataset_mean_std.txt', 'a')
	total_mean=[]
	
	total_mean[0]= mean_max  / num_files
	total_mean[1]= mean_mean / num_files
	total_mean[2]= mean_min  / num_files
	total_mean[3]= mean_numb / num_files
	total_mean[4]= mean_std  / num_files
	texto =("\nDataset Mean - Map_max, Map_mean, Map_min, Map_numb, Map_std: \n {:.10f} {:.10f} {:.10f} {:.10f} {:.10f} ".format(total_mean_max, total_mean_mean, total_mean_min, total_mean_numb, total_mean_std))
	arq.write(texto)
	print("\nDataset Mean - Map_max, Map_mean, Map_min, Map_numb, Map_std: ")
	print(total_mean_max, total_mean_mean, total_mean_min, total_mean_numb, total_mean_std)
	
	return total_mean


def calculate_std(sum_var_max, sum_var_mean, sum_var_min, sum_var_numb, sum_var_std, total_elements, num_files):
	
	arq = open(out_path + 'dataset_mean_std.txt', 'a')
	
	var_max = sum_var_max  / (total_elements)
	var_mean= sum_var_mean / (total_elements)
	var_min = sum_var_min  / (total_elements)
	var_numb= sum_var_numb / (total_elements)
	var_std = sum_var_std  / (total_elements)
	texto =("\nDataset Variance - Map_max, Map_mean, Map_min, Map_numb, Map_std: \n {:.10f} {:.10f} {:.10f} {:.10f} {:.10f} ".format(var_max, var_mean, var_min, var_numb, var_std))
	arq.write(texto)
	print("\nDataset Variance - Map_max, Map_mean, Map_min, Map_numb, Map_std: ")	
	print(var_max, var_mean, var_min, var_numb, var_std)								
	
	std_max  = math.sqrt(var_max)
	std_mean = math.sqrt(var_mean)
	std_min  = math.sqrt(var_min)
	std_numb = math.sqrt(var_numb)
	std_std  = math.sqrt(var_std)
	texto = ("\nDataset Standard Deviation- Map_max, Map_mean, Map_min, Map_numb, Map_std: \n {:.10f} {:.10f} {:.10f} {:.10f} {:.10f}".format(std_max, std_mean, std_min, std_numb, std_std))
	arq.write(texto)
	print("\nDataset Standard Deviation- Map_max, Map_mean, Map_min, Map_numb, Map_std: ")
	print(std_max, std_mean, std_min, std_numb, std_std)
	
	arq.close()

def load_cut_rotate_and_save_dataset(item, input_dimensions, new_size, num_rotations, radius, rotate_dataset):
	data = load_statistics_label_view(item)
	new_data = np.zeros((input_dimensions, new_size, new_size))
	new_data = circle_and_mean(data, radius, new_size)
	if rotate_dataset:
		rots = rotate(new_data, num_rotations)
	else:
		rots = None
						
	save_as_sparse_matrix(new_data, rots, new_size, item)
			
##############################################


#main
dataset_list = getDatasetList(dataset_list_file)

num_files = len(dataset_list)
   
    
inicio = time.time()

mythreads = []
total = 0

for item in dataset_list:
	t = multiprocessing.Process(target=load_cut_rotate_and_save_dataset, args=[item, input_dimensions, new_size, num_rotations, radius, rotate_dataset])
	mythreads.append(t)
	t.start()
	total += 1
#	print("Peguei a thread {} \n estou com o item {} \n\n".format(total, item))
	if (len(mythreads) >= max_threads) or not (total < num_files):
		for t in mythreads:
			t.join()
		mythreads[:] = []  # clear the thread's list
		print('Saved {} images already...'.format(total))

fim = time.time()
print("\n\n\nTempo total: ", fim - inicio)
