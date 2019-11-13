import os
import sys
import torch
import model_inference as M
import numpy as np
# from PIL import Image
import cv2
from torchvision import datasets, transforms
# from posix import wait

# 
# 

TRANSFORMS = transforms.Normalize([0.0128, 0.0119, 0.0077, 0.0019, 0.0010], [0.0821, 0.0739, 0.0591, 0.0170, 0.0100])
device = torch.device("cuda:0")
carmen_home = os.getenv("CARMEN_HOME")
model_path = '50.model'
input_channels = 5
n_classes = 3
dropout_prob = 0.0
model = M.FCNN(n_input=input_channels, n_output=n_classes, prob_drop=dropout_prob).to(device)
model.load_state_dict(torch.load(model_path))
model.eval()
# images_converted = 0
# model = M.FCNN(n_output=3)
# model.load_state_dict(torch.load(saved_path))
# model = model.eval()

# def png2tensor(img):
#     img2tensor = transforms.ToTensor()
#     return img2tensor(img)
# 
# def tensor2png(tensor):
#     trans = transforms.ToPILImage()
#     return trans(tensor)
# 
# 
# def save_to_debug(data, imgPred):
#     print("aqui foi\n")
#     imgData = data[0][3].numpy()
#     img2 = (imgData)*255/3
#     img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
#     img_map = np.zeros((600,600,3), np.uint8)
#     img_map[np.where(((img2 != [0])).all(axis = 2))] = np.array([0,255,0])
#     imPredShow = tensor2rgbimage(imgPred)
#     imgs_comb = np.hstack((img_map, imPredShow))
#     cv2.imwrite(str(images_converted) + '_Combined.png',  imgs_comb)
#     
# 
# def tensor2rgbimage(tensor):
#     img = tensor.permute(1,2,0).numpy()
#     height, width = img.shape[:2]
#     img_map = np.zeros((width,height,3), np.uint8)
#     #print(np.where((img == [1]).all(axis = 2)))
#     img_map[np.where((img == [0]).all(axis = 2))] = np.array([255,120,0])
#     img_map[np.where((img == [1]).all(axis = 2))] = np.array([255,255,255])
#     img_map[np.where((img == [2]).all(axis = 2))] = np.array([0,0,0])
# 
#     return img_map
# 
def fixed_normalize_cell(value_map, new_max, last_max, min):
    new_val = value_map
    if(new_val < min):
        new_val = min
    normalized = (new_val-min)*(new_max)/(last_max-min)
    #printf("max = %lf | min = %lf | norm_max = %d | norm_min = %d\n", max, min, normalized[max_index], normalized[min_index]);
    return normalized

def normalize_maps():
    for i in range(size):
        for j in range(size):
            new_data[0][i][j] = fixed_normalize_cell(data[0][i][j], 1.0, 1.852193, map_min) #max
            new_data[1][i][j] = fixed_normalize_cell(data[1][i][j], 1.0, 1.852193, map_min) #mean
            new_data[2][i][j] = fixed_normalize_cell(data[2][i][j], 1.0, 1.852193, map_min) #min
            new_data[3][i][j] = fixed_normalize_cell(data[3][i][j], 1.0, 64,        map_min) #numb
            new_data[4][i][j] = fixed_normalize_cell(data[4][i][j], 1.0, 15,       map_min) #std
    return new_data
            

def labels_to_img(numpy_image, size):
    reshaped = numpy_image
    img = np.zeros((size, size, 3), np.uint8)
    for i in range(size):
        # print("")
        for j in range(size):
            if reshaped[i][j] == 0:
                img[i][j] = np.array([255, 120, 0])
            elif reshaped[i][j] == 1.0:
                img[i][j] = np.array([255, 255, 255])
            elif reshaped[i][j] == 2.0:
                img[i][j] = np.array([0, 0, 0])
    return img


def convert_metric_map_to_image(metric_map, metric_type, size):
# TODO Pegar das variáveis
    map_max = 1
    map_min = 0
    img = fixed_normalize_to_img(metric_map, 255.0, map_max, map_min, size)
    return img


def get_raw_min_and_max_values_to_normalize(metric_type):
    map_min = -1.0
    if metric_type == 'max':
        map_max = 1.852193
    elif metric_type == 'numb':
        map_max = 64.0
        map_min = 0.0
    elif metric_type == 'min':
        map_max = 1.852193
    elif metric_type == 'mean':
        map_max = 1.852193
    elif metric_type == 'std':
        map_max = 15.0
        map_min = -1.0

    return map_max, map_min


def convert_raw_metric_map_to_image(metric_map, metric_type, size):
# TODO Pegar das variáveis
    map_max, map_min = get_raw_min_and_max_values_to_normalize(metric_type)
    img = fixed_normalize_to_img(metric_map, 255.0, map_max, map_min, size)
    return img

def fixed_normalize_to_img(map, new_max_value, max_value, min, size):
    img_map = np.zeros((size, size, 3), np.uint8)

    for i in range(size):
        for j in range(size):
            new_val = map[j][i]
#             print("{:f}".format(new_val))
            if new_val < min:
                img_map[j][i][0] = min
                img_map[j][i][1] = min
                img_map[j][i][2] = min
            cel_pixel = int((new_val - min) * new_max_value / (max_value - min))
            img_map[j][i][0] = cel_pixel
            img_map[j][i][1] = cel_pixel
            img_map[j][i][2] = cel_pixel
    return img_map


def file_to_numpy(img_x_dim, img_y_dim, file):
    reshaped = numpy_file.reshape(img_x_dim, img_y_dim)
    return reshaped


def load_bach_predict_data(transforms, map_max, map_mean, map_min, map_numb, map_std, input_dimensions, img_x_dim, img_y_dim, n_classes):

    data = torch.zeros(1, input_dimensions, img_x_dim, img_y_dim)
    
    data[0][0] = torch.from_numpy(map_max.reshape(img_x_dim, img_y_dim))
    data[0][1] = torch.from_numpy(map_mean.reshape(img_x_dim, img_y_dim))
    data[0][2] = torch.from_numpy(map_min.reshape(img_x_dim, img_y_dim))
    data[0][3] = torch.from_numpy(map_numb.reshape(img_x_dim, img_y_dim))
    data[0][4] = torch.from_numpy(map_std.reshape(img_x_dim, img_y_dim))
    
    if transforms != None:
        data[0] = transforms(data[0])
        
#         cv2.imshow('window', labels_to_img(target[j].numpy(), img_x_dim))
#         cv2.waitKey(0)
        # batch_weight = batch_weight + new_weights
    # cv2.imshow('Max', convert_metric_map_to_image(data[0][0], 'max', size))
    return data


def process_image(map_max, map_mean, map_min, map_numb, map_std):
    global TRANSFORMS
    global input_channels
    global n_classes
    global model_path
    global model
    
    h = 600
    w = 600
    
    print("Testing!")
#     map_max =  map_max.reshape(h,w)
#     map_mean = map_mean.reshape(h,w)
#     map_min =  map_min.reshape(h,w)
#     map_numb = map_numb.reshape(h,w)
#     map_std =  map_std.reshape(h,w)
    print('Using Device: ', device)
    data = load_bach_predict_data(TRANSFORMS, map_max, map_mean, map_min, map_numb, map_std, input_channels, w, h, n_classes)
#     cv2.imshow('Max', convert_metric_map_to_image(data[0][0], 'max', w))
#     cv2.imshow('Mean', convert_metric_map_to_image(data[0][1], 'mean', w))
#     cv2.imshow('Min', convert_metric_map_to_image(data[0][2], 'min', w))
#     cv2.imshow('Numb', convert_metric_map_to_image(data[0][3], 'numb', w))
#     cv2.imshow('Std', convert_metric_map_to_image(data[0][4], 'std', w))
#     cv2.waitKey(100)
    
    with torch.no_grad():
        data = data.to(device)
        output, prob_softmax = model(data)
#         preditction = output
#         predicted_map = preditction[0].cpu()
       
        pred = prob_softmax.max(1, keepdim=True)[1] # get the index of the max log-probability
        imgPred = pred[0]
        imgPred = imgPred.cpu().float()
        
    cv2.imshow("No Python", labels_to_img(imgPred[0].numpy(), h))
    cv2.waitKey(100)
    result = prob_softmax.cpu()
    result2 = result[0].permute(1,2,0)
    print(result2.numpy().dtype)
    print(result2.numpy().shape)
#     print(result2[0,0,:3])
    
#     cv2.imshow("No Python", result.numpy())
    
    return (result2.numpy().astype(np.float64).flatten())
################Para Debug












