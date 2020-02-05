from __future__ import print_function, division
import os
import os.path
from config import *
from utils import *
from model import *
from segmenter import SegmenterNet
import tensorflow as tf
import numpy as np
import cv2
import matplotlib.pyplot as plt

global net
global cfg
global sess

#cfg = lidar_config()
#os.environ['CUDA_VISIBLE_DEVICES'] = str(cfg.GPU)
#print("creating network model using gpu " + str(cfg.GPU))

'''Initialize tensorflow and model'''
def initialize(vertical_resolution):
    global net
    global cfg
    global sess

    # get config params
    cfg = lidar_config()
    #cfg.IMAGE_WIDTH = 512          # image width
    #cfg.IMAGE_HEIGHT = 32          # image height
    # get trained model checkpoints
    model_ckp_name = os.getenv("CARMEN_HOME") + "/sharedlib/salsanet/logs/salsaNet_trained/model.chk-300"

    # load the trained model
    net = SegmenterNet(cfg, model_ckp_name)

    '''Loads tensorflow '''
    tf.Graph().as_default()
    sess = tf.Session(config=tf.ConfigProto(allow_soft_placement=True), graph=net.graph)
    net.initialize_vars(sess)
    
    print ("\n\n-------------------------------------------------------")
    print ("       SalsaNet: Pretrained Model and Tensorflow loaded!")
    print ("-------------------------------------------------------\n\n")

def showPredImg(frontalimg=[], rearimg=[]):
    img = np.concatenate((rearimg, frontalimg),axis=1)
    showBevImg(img)

def showBevImg(img=[]):
    plt.imshow(img)
    plt.axis('off')
    plt.show(block=False)
    plt.pause(2)
    plt.close()

def salsanet_process_point_cloud(lidar, timestamp):
    global sess
    global net
    global cfg
    print("lidar.shape={}".format(
        lidar.shape)) #(99928, 7)
    #lidar = vertical_interpolation(lidar)
    
    AZIMUTH_LEVEL = lidar.shape[1]
    ZENITH_LEVEL = lidar.shape[0]

    lidar_mask = np.reshape(
            (lidar[:, :, 4] > 0),
            [ZENITH_LEVEL, AZIMUTH_LEVEL, 1])
    lidar = np.append(lidar, lidar_mask, axis=2)
    
    lidar = lidar.reshape((AZIMUTH_LEVEL * ZENITH_LEVEL,6))
    #print("lidar.shape={}".format(
    #    lidar.shape))
    # define the region of interest for bird eye view image generation
    #Frontal View
    frontalView = PC2ImgConverter(imgChannel=cfg.IMAGE_CHANNEL, xRange=[0, 50], yRange=[-12, 12], zRange=[-10, 8],
                                                   xGridSize=0.2, yGridSize=0.3, zGridSize=0.3, maxImgHeight=cfg.IMAGE_HEIGHT,
                                                   maxImgWidth=cfg.IMAGE_WIDTH, maxImgDepth=64)
    #Rear View - Not right yet - needs correction
    rearView = PC2ImgConverter(imgChannel=cfg.IMAGE_CHANNEL, xRange=[-50, 0], yRange=[-12, 12], zRange=[-10, 8],
                                                   xGridSize=0.2, yGridSize=0.3, zGridSize=0.3, maxImgHeight=cfg.IMAGE_HEIGHT,
                                                   maxImgWidth=cfg.IMAGE_WIDTH, maxImgDepth=64)
    

    bevFrontalImg, bevFrontalCloud = frontalView.getBEVImage(lidar)
    bevFrontalImg = bevFrontalImg.astype('float32') / 255
    bevRearImg, bevRearCloud = rearView.getBEVImage(lidar)
    bevRearImg = bevRearImg.astype('float32') / 255
    #showBevImg(bevRearImg)
    
    pred_frontal_img = net.predict_single_image(input_img=bevFrontalImg, session=sess)
    pred_rear_img = net.predict_single_image(input_img=bevRearImg, session=sess)
    print('predicted image shape: ', pred_frontal_img.shape, ' type: ', pred_frontal_img.dtype, ' min val: ', pred_frontal_img.min(),
          ' max val: ', pred_frontal_img.max())
    s = (AZIMUTH_LEVEL * ZENITH_LEVEL)
    pointCloudSegmented = np.zeros(s, dtype=np.int64)
    frontalView.getCloudsFromAnyImage(pred_frontal_img, lidar, pointCloudSegmented) 
    rearView.getCloudsFromAnyImage(pred_rear_img, lidar, pointCloudSegmented)

    pointCloudSegmented = pointCloudSegmented.reshape((ZENITH_LEVEL, AZIMUTH_LEVEL))
    #print('pointCloudSegmented.shape=', pointCloudSegmented.shape)
    #showPredImg(pred_frontal_img, pred_rear_img)
    #showBevImg(pred_rear_img)
    return pointCloudSegmented