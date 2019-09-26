
from __future__ import print_function
import argparse
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.legacy.nn as nn2
import torch.optim as optim
from torchvision import datasets, transforms
from PIL import Image
import math
import model as M
from random import randint
import train as train
import time
from random import shuffle
import numpy as np
import cv2

img_index = 1


img_x_dim = 600
img_y_dim = 600

data_dim = 5

data_path = '/media/vinicius/NewHD/Datasets/Neural_Mapper_dataset/60mts-180907_rot/data/'
target_path = '/media/vinicius/NewHD/Datasets/Neural_Mapper_dataset/60mts-180907_rot/labels/'
debug_img_path = '/media/vinicius/NewHD/Datasets/Neural_Mapper_dataset/60mts-180907_rot/debug_imgs/'


def getDatasetList(file_name):
    file = open(file_name)
    content = file.read()
    tmp = content.split('\n')
    content_list = list(filter(None, tmp))
    return content_list


def load_image(index):
    data = torch.zeros(1, data_dim, img_x_dim, img_y_dim)
    target = torch.zeros(1, img_x_dim, img_y_dim)

    data[0][0] = train.png2tensor(data_path + str(index) + '_max.png')[0]
    data[0][1] = train.png2tensor(data_path + str(index) + '_mean.png')[0]
    data[0][2] = train.png2tensor(data_path + str(index) + '_min.png')[0]
    data[0][3] = train.png2tensor(data_path + str(index) + '_numb.png')[0]
    data[0][4] = train.png2tensor(data_path + str(index) + '_std.png')[0]
    target[0] = train.png2tensor(target_path + str(index) + '_view.png')[0]

    return data, target

if __name__ == '__main__':
    # Training settings
    parser = argparse.ArgumentParser(description='PyTorch Neural Mapper test on images')
    parser.add_argument('--img-index', type=int, default=randint(0,1840), metavar='N',
                        help='Image index on dataset')
    parser.add_argument('--save-img', type=bool, default=True, metavar='N',
                        help='Save image on disk')
    parser.add_argument('--model-name', type=str, default='10.model', metavar='N',
                        help='Save image on disk')
    parser.add_argument('--test-file', type=str, default='teste.txt', metavar='N',
                        help='File with the test set')

    args = parser.parse_args()
    device = torch.device("cuda:0")
    model = M.FCNN(n_output=3)
    model.load_state_dict(torch.load(args.model_name))
    model = model.eval()
    text = "Set de Treino + Validacao"

    indexes = getDatasetList(args.test_file)
    num_img = len(indexes)
    # for i in range(num_img):
    #     indexes.append(i)
    #print (indexes)

    #shuffle(indexes)
    for i in indexes:
#    data, target = load_image(args.img_index)
        print(str(i)+ ": ")
        data, target = load_image(i)
        #start = time.now()
        output = model(data)
        pred = output.max(1, keepdim=True)[1] # get the index of the max log-probability

        imgPred = pred[0].float()
        imgPred = (imgPred + 1)
        imgPred = imgPred.cpu().float()

        # imgTarget[0] = target[0]
        # imgTarget = imgTarget.cpu().float()
        if(args.save_img):
            #train.saveImage(imgPred, debug_img_path + '/predic_epoch' + args.model_name.split('.')[0] + 'img' + str(args.img_index) + '.png')
            #train.saveImage(imgTarget, debug_img_path + '/target_epoch' + args.model_name.split('.')[0] + 'img' + str(args.img_index) + '.png')

            train.saveImage(imgPred, debug_img_path + i + '_PREDICT.png')
            #train.saveRGBImage(imgPred, debug_img_path + i + '_PREDICT.png')
            imgTarget = np.zeros((img_x_dim, img_y_dim, 3), np.uint8)
            imgTarget = cv2.imread(target_path + str(i) + '_view.png')
            cv2.imwrite(debug_img_path + i + '_TARGET.png', imgTarget)
            #train.saveRGBImage(imgTarget, debug_img_path + i + '_TARGET.png')
            #        if(i > 1000):
            #           text = "Set de Teste"
        #train.showOutput(imgPred)
        #train.showOutput(imgTarget)
        #print(imgPred)
            #print(imgTarget)
            #print(output.size())
