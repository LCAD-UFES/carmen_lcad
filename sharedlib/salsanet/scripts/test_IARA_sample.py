from __future__ import print_function, division
import os.path
from config import *
from utils import *
from model import *
from segmenter import SegmenterNet
import numpy as np

import matplotlib.pyplot as plt

cfg = lidar_config()
os.environ['CUDA_VISIBLE_DEVICES'] = str(cfg.GPU)
print("creating network model using gpu " + str(cfg.GPU))

def test_with_point_cloud(net, cfg, cloud_path):
    
    # load pc
    lidar = np.loadtxt(cloud_path, delimiter="\t")
    print("lidar.shape={}".format(
        lidar.shape)) #(99928, 7)
    # minInColumns = np.amin(lidar, axis=0)
    # print('lidar: min value of every column: ', minInColumns)
    # maxInColumns = np.amax(lidar, axis=0)
    # print('lidar: max value of every column: ', maxInColumns)

    AZIMUTH_LEVEL = lidar.shape[0]//cfg.IMAGE_HEIGHT
    lidar = lidar.reshape((cfg.IMAGE_HEIGHT,AZIMUTH_LEVEL,5))
    #save_txt(lidar, "original", "_iara")
    lidar_mask = np.reshape(
            (lidar[:, :, 4] > 0),
            [cfg.IMAGE_HEIGHT, AZIMUTH_LEVEL, 1])
    print(lidar_mask.shape)
    lidar = np.append(lidar, lidar_mask, axis=2)

    lidar = lidar.reshape((AZIMUTH_LEVEL*cfg.IMAGE_HEIGHT,6))
    # define the region of interest for bird eye view image generation
    #Frontal View
    frontalView = PC2ImgConverter(imgChannel=cfg.IMAGE_CHANNEL, xRange=[0, 50], yRange=[-3, 6], zRange=[-10, 8],
                                                   xGridSize=0.2, yGridSize=0.3, zGridSize=0.3, maxImgHeight=cfg.IMAGE_HEIGHT,
                                                   maxImgWidth=cfg.IMAGE_WIDTH, maxImgDepth=64)
    #Rear View
    rearView = PC2ImgConverter(imgChannel=cfg.IMAGE_CHANNEL, xRange=[-50, 0], yRange=[-3, 6], zRange=[-10, 8],
                                                   xGridSize=0.2, yGridSize=0.3, zGridSize=0.3, maxImgHeight=cfg.IMAGE_HEIGHT,
                                                   maxImgWidth=cfg.IMAGE_WIDTH, maxImgDepth=64)
    
    testView = PC2ImgConverter(imgChannel=cfg.IMAGE_CHANNEL, xRange=[0, 100], yRange=[-10, 10], zRange=[-10, 10],
                                                   xGridSize=0.1, yGridSize=0.1, zGridSize=0.1, maxImgHeight=cfg.IMAGE_HEIGHT*4,
                                                   maxImgWidth=cfg.IMAGE_WIDTH*4, maxImgDepth=64)
    bevTestImg, bevTestCloud = testView.getBEVImage(lidar)
    showBevImg(bevTestImg)

    bevFrontalImg, bevFrontalCloud = frontalView.getBEVImage(lidar)
    bevFrontalImg = bevFrontalImg.astype('float32') / 255
    bevRearImg, bevRearCloud = rearView.getBEVImage(lidar)
    bevRearImg = bevRearImg.astype('float32') / 255
    #print('bevCloud shape: ', bevCloud.shape) # (42419, 7)
    #print('bird eye view image frontal shape: ', bevFrontalImg.shape) # (64, 512, 4)

    with tf.Session(graph=net.graph) as sess:
        net.initialize_vars(sess)
        net.predict
        pred_frontal_img = net.predict_single_image(input_img=bevFrontalImg, session=sess)
        pred_rear_img = net.predict_single_image(input_img=bevRearImg, session=sess)
        print('predicted image shape: ', pred_frontal_img.shape, ' type: ', pred_frontal_img.dtype, ' min val: ', pred_frontal_img.min(),
              ' max val: ', pred_frontal_img.max())
        
        #roadCloud, vehicleCloud = pc2img.getCloudsFromBEVImage(pred_img, bevCloud, postProcessing=True)
        showPredImg(pred_frontal_img, pred_rear_img)

def showPredImg(frontalimg=[], rearimg=[]):
    img = np.concatenate((rearimg, frontalimg),axis=1)
    showBevImg(img)

def showBevImg(img=[]):
    plt.imshow(img)
    plt.axis('off')

    plt.show()


def main():

    # get config params
    cfg = lidar_config()
    
    # get trained model checkpoints
    model_ckp_name = "../logs/salsaNet_trained/model.chk-300"

    # load the trained model
    net = SegmenterNet(cfg, model_ckp_name)

    # testing by using the raw point cloud data
    test_cloud = "../data/1542360705.470126.txt"
    test_with_point_cloud(net, cfg, test_cloud)

    return

if __name__ == '__main__':

    main()
