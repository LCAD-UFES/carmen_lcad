
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

img_index = 1


img_x_dim = 500
img_y_dim = 500

data_dim = 5

data_path = '/dados/neural_mapper_png_dataset/volta_da_ufes30102018/data/'
target_path = '/dados/neural_mapper_png_dataset/volta_da_ufes30102018/labels/'
debug_img_path = 'debug_imgs/'

def load_image(index):
    data = torch.zeros(1, data_dim, img_x_dim, img_y_dim)
    target = torch.zeros(1, img_x_dim, img_y_dim)

    data[0][0] = train.png2tensor(data_path + str(index) + '_max.png')
    data[0][1] = train.png2tensor(data_path + str(index) + '_mean.png')
    data[0][2] = train.png2tensor(data_path + str(index) + '_min.png')
    data[0][3] = train.png2tensor(data_path + str(index) + '_numb.png')
    data[0][4] = train.png2tensor(data_path + str(index) + '_std.png')
    target[0] = train.png2tensor(target_path + str(index) + '_view.png')

    return data, target

if __name__ == '__main__':
    # Training settings
    parser = argparse.ArgumentParser(description='PyTorch Neural Mapper test on images')
    parser.add_argument('--img-index', type=int, default=randint(0,400), metavar='N',
                        help='Image index on dataset')
    parser.add_argument('--save-img', type=bool, default=False, metavar='N',
                        help='Save image on disk')
    parser.add_argument('--model-name', type=str, default='10.model', metavar='N',
                        help='Save image on disk')

    args = parser.parse_args()
    model = M.FCNN(n_output=3)
    model.load_state_dict(torch.load('saved_models/'+args.model_name))
    model = model.eval()
    
    data, target = load_image(args.img_index)
    output = model(data)
    pred = output.max(1, keepdim=True)[1] # get the index of the max log-probability
    imgPred = pred[0].float()
    imgPred = (imgPred + 1)*255/3
    imgPred = imgPred.cpu().float()
    imgTarget = torch.FloatTensor(1, img_x_dim, img_y_dim)
    imgTarget[0] = target[0]
    imgTarget = imgTarget.cpu().float()
    if(args.save_img):
        train.saveImage(imgPred, debug_img_path + '/predic_epoch' + args.model_name.split('.')[0] + 'img' + str(args.img_index) + '.png')
        train.saveImage(imgTarget, debug_img_path + '/target_epoch' + args.model_name.split('.')[0] + 'img' + str(args.img_index) + '.png')
    train.showOutput(imgPred)
    train.showOutput(imgTarget)
    print(imgPred)
    print(imgTarget)
    print(output.size())
