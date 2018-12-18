    
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
import cv2
import numpy as np
from cv2 import waitKey


img_index = 1


img_x_dim = 600
img_y_dim = 600

data_dim = 5

data_path = '/media/vinicius/NewHD/Datasets/Neural_Mapper_dataset/60mts_guarapari_circ/data/'
target_path = '/media/vinicius/NewHD/Datasets/Neural_Mapper_dataset/60mts_guarapari_circ/labels/'
debug_img_path = 'debug_imgs/'


if __name__ == '__main__':
    # Training settings
    parser = argparse.ArgumentParser(description='PyTorch Neural Mapper test on images')
    parser.add_argument('--img-index', type=int, default=randint(0,1840), metavar='N',
                        help='Image index on dataset')
    parser.add_argument('--save-img', type=bool, default=False, metavar='N',
                        help='Save image on disk')
    parser.add_argument('--model-name', type=str, default='10.model', metavar='N',
                        help='Save image on disk')

    device = torch.device("cuda")

    args = parser.parse_args()
    model = M.FCNN(n_output=3).to(device)
    model.load_state_dict(torch.load('saved_models/'+args.model_name))
    model = model.eval()
    text = "Set de Treino + Validacao"


    #print (indexes)
    dataset_list = train.getDatasetList("teste_guarapari_filtred.txt")
    print(dataset_list)
    #shuffle(indexes)
    for i in range(len(dataset_list)):
#    data, target = load_image(args.img_index)
        print(str(i)+ ": ")
        data, target, weights = train.load_data2(1, i, dataset_list, data_path, target_path)
        data, target = data.to(device), target.long().to(device)

        start = time.time()
        output = model(data)
        print(time.time() - start)

        pred = output.max(1, keepdim=True)[1] # get the index of the max log-probability
        
        imgPred = pred[0]
        imgPred = imgPred.cpu().float()
        imgTarget = torch.FloatTensor(1, img_x_dim, img_y_dim)
        imgTarget[0] = target[0]
        imgTarget = imgTarget.cpu().float()
        if(args.save_img):
            #train.saveImage(imgPred, debug_img_path + '/predic_epoch' + args.model_name.split('.')[0] + 'img' + str(args.img_index) + '.png')
            #train.saveImage(imgTarget, debug_img_path + '/target_epoch' + args.model_name.split('.')[0] + 'img' + str(args.img_index) + '.png')
            train.saveImage2(imgPred, debug_img_path + str(i) + '_PREDICT.png')
            train.saveImage2(imgTarget, debug_img_path + str(i) + '_TARGET.png')
            #        if(i > 1000):
        #           text = "Set de Teste"
        train.showOutput(imgPred)
        
'''        
        imgData = data[0][3].numpy()
        img2 = (imgData)*255/3
#         print((data_path + dataset_list[i] + '_mean.png'))
#         imgData = cv2.imread((data_path + dataset_list[i] + '_num.png'))
        
        img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
        img_map = np.zeros((600,600,3), np.uint8)
        img_map[np.where(((img2 != [0])).all(axis = 2))] = np.array([0,255,0])
#         img2 = np.zeros_like(imgData)
#         img2 = cv2.merge((imgData,imgData,imgData))
        
        imPredShow = train.tensor2rgbimage(imgPred)
        imTargetShow = train.tensor2rgbimage(imgTarget)
#         imgs = [imgData.data, imPredShow.data, imTargetShow.data]
        
        imgs_comb = np.hstack((img_map, imPredShow, imTargetShow))
#         img_map[np.where(((imgs_comb != [255,255,255]) and (imgs_comb != [0,0,0])).all(axis = 2))] = np.array([255,120,0])
#         img_map[np.where((imgs_comb != [255,120,0]).all(axis = 2))] = np.array([255,120,0])
         
#         cv2.imshow('Window2',imTargetShow)
#         cv2.imshow('Window',imgs_comb)
        cv2.imwrite(debug_img_path + str(i) + '_Combined.png',  imgs_comb)
#         cv2.imshow('Window',img_map)
#         cv2.waitKey(1)
#         train.showOutput2(imgPred, "pred")
#         train.showOutput2(imgTarget, "targ")
        #print(imgPred)
        #print(imgTarget)
        #print(output.size())
'''
