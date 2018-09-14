
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
import train

img_index = 1


img_x_dim = 424
img_y_dim = 424

data_dim = 5


def load_image(index, data_path, target_path):
    data = torch.zeros(1, data_dim, img_x_dim, img_y_dim)
    target = torch.zeros(1, img_x_dim, img_y_dim)

    data[0][0] = train.png2tensor(data_path + str(index + 1) + '_max.png')
    data[0][1] = train.png2tensor(data_path + str(index + 1) + '_mean.png')
    data[0][2] = train.png2tensor(data_path + str(index + 1) + '_min.png')
    data[0][3] = train.png2tensor(data_path + str(index + 1) + '_numb.png')
    data[0][4] = train.png2tensor(data_path + str(index + 1) + '_std.png')
    target[0] = train.png2tensor(target_path + str(index) + '_view.png')

    return data, target

if __name__ == '__main__':
    # Training settings
    parser = argparse.ArgumentParser(description='PyTorch Neural Mapper test on images')
    parser.add_argument('--img-index', type=int, default=randint(0,78), metavar='N',
                        help='Image index on dataset')

    parser.add_argument('--save-img', type=bool, default=False, metavar='N',
                        help='Save image on disk')

    parser.add_argument('--model-name', type=str, default='10.model', metavar='N',
                        help='Save image on disk')

    parser.add_argument('--input-path', type=str, default='/dados/neural_mapper/60mts/data/', metavar='N',
                        help='Input images path')

    parser.add_argument('--target-images', type=str, default='/dados/neural_mapper/60mts/labels/', metavar='N',
                        help='Ground truth images path to compare with predicted')

    parser.add_argument('--output-path', type=str, default='/dados/neural_mapper/60mts/debug_imgs/', metavar='N',
                        help='Output path for predicted images and gt')

    args = parser.parse_args()

    data_path = args.input_path
    target_path = args.target_images
    debug_img_path = args.output_path

    model = M.FCNN()
    model.load_state_dict(torch.load(args.model_name))

    data, target = load_image(args.img_index, data_path, target_path)
    output = model(data)
    pred = output.max(1, keepdim=True)[1] # get the index of the max log-probability
    imgPred = pred[0]
    imgPred = imgPred.cpu().float()
    imgTarget = torch.FloatTensor(1, 424, 424)
    imgTarget[0] = target[0]
    imgTarget = imgTarget.cpu().float()
    if(args.save_img):
        train.saveImage(imgPred, debug_img_path + '/predic_epoch/' + 'img' + str(args.img_index) + '.png')
        train.saveImage(imgTarget, debug_img_path + '/target_epoch/' + 'img' + str(args.img_index) + '.png')
    train.showOutput(imgPred)
    train.showOutput(imgTarget)

