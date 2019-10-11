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
from numpy import std, dtype, float64
import time

import scipy.sparse
from PIL import Image
import pandas as pd
import sys

import torch
from torch.utils.data import Dataset
from torch.utils.data.dataloader import DataLoader


# import torch.nn as nn
# import torch.optim as optim

# from torchvision import models, transforms

BATCH_SIZE = 5105

DATASET_SIZE = 5105


dataset_list_file = "/home/lcad/viniciusbc/train_list.txt"
dataset_path = "/home/lcad/viniciusbc/"
data_path = "data/"

#statistics + label + view
input_dimensions = 5

new_count = 0
size = 600
center = 600/2
velodyne_max_range = 60
map_resolution = 0.2
radius = (velodyne_max_range/map_resolution)
new_size = 600
rotate_dataset = True


def getDatasetList(file_name):
    file = open(file_name)
    content = file.read().splitlines()
    return content

# 
# def file_to_numpy(img_x_dim, img_y_dim, file):
#     numpy_file = np.fromfile(file, dtype=float)
#     reshaped = numpy_file.reshape(img_x_dim, img_y_dim)
#     return reshaped


def file_npz_to_numpy(file):
    sparse_matrix = scipy.sparse.load_npz(file)
    normalized_data = sparse_matrix.todense()
    data = torch.from_numpy(normalized_data)
    return data


def load_statistics_label_view(item):
    data_files = dataset_path + data_path + str(item)
    data = torch.zeros((BATCH_SIZE, input_dimensions, size, size))
    
    for i in range(DATASET_SIZE):
#     print(data_files)
        data[i][0] = (file_npz_to_numpy(data_files + '_max.npz'))
        data[i][1] = (file_npz_to_numpy(data_files + '_mean.npz'))
        data[i][2] = (file_npz_to_numpy(data_files + '_min.npz'))
        data[i][3] = (file_npz_to_numpy(data_files + '_numb.npz'))
        data[i][4] = (file_npz_to_numpy(data_files + '_std.npz'))
#     for i in data:
#         print(i)
#     exit()
    return data

def calculate_mean_in_one_shot(data):
#     mean = torch.empty(5, dtype=torch.float64)
    data = torch.transpose(data, 0,1).contiguous()
    data = data.view(data.size(0), -1)
    mean = data.mean(1)
    std_dev = data.std(1)
    print(data.shape)
    print(mean.shape)
    print(std_dev.shape)
    
    return (mean / (DATASET_SIZE/BATCH_SIZE)), std_dev


def calculate_std_in_one_shot(data):
    
    std_dev = torch.empty(5, dtype=torch.float64)
    print('std')
    std_dev = torch.std(data, dim=1)
     
    return std_dev

#main
dataset_list = getDatasetList(dataset_list_file)

num_files = len(dataset_list)

inicio = time.time()

for item in dataset_list:
    data = load_statistics_label_view(item)

mean_here, std_here = calculate_mean_in_one_shot(data)
print("\nMean: ", mean_here)
# std_here = calculate_std_in_one_shot(data)
print("\nSTD: ", std_here)
fim = time.time()
print("\n\n\nTempo total: ", fim - inicio)
