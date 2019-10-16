from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from datetime import datetime

import os
import os.path
import sys
import time
import glob
import cv2

import numpy as np
from six.moves import xrange
import tensorflow as tf
from PIL import Image

from config import *
from imdb import kitti
from utils.util import *
from nets import *
from setuptools.sandbox import ExceptionSaver
from IN import MCAST_BLOCK_SOURCE

'''Defining global variables'''
global mc
global model
global sess

'''Initialize tensorflow and model within specific vertical resolution and number shots to squeeze'''
def initialize(vertical_resolution, shots_to_squeeze):
    global mc
    global model
    global sess
    
    '''Loads squeezeseg config and changes the zenith and azimuth level'''
    mc = kitti_squeezeSeg_config()
    mc.ZENITH_LEVEL = vertical_resolution
    #mc.AZIMUTH_LEVEL = shots_to_squeeze
 
    mc.LOAD_PRETRAINED_MODEL = False
    mc.BATCH_SIZE = 1
    model = SqueezeSeg(mc)
     
    '''Loads tensorflow'''
    tf.Graph().as_default()
    sess = tf.Session(config=tf.ConfigProto(allow_soft_placement=True))
    saver = tf.train.Saver(model.model_params)
    saver.restore(sess, os.getenv("CARMEN_HOME") + '/sharedlib/libsqueeze_seg_v2/data/SqueezeSegV2/model.ckpt-30700')  
     
    print ("\n\n-------------------------------------------------------")
    print ("       Pretrained Model and Tensorflow loaded!")
    print ("-------------------------------------------------------\n\n")


def _normalize(x):
    return (x - x.min())/(x.max() - x.min())

def generate_lidar_images(lidar, pred_cls):
    global mc
    alpha = 0.4
    beta = (1.0 - alpha)
    src1 = (255 * _normalize(lidar[:, :, 3])).astype(np.uint8)
    src2 = (255 * visualize_seg(pred_cls, mc)[0]).astype(np.uint8)
    dst = np.uint8(alpha*(src1[:,:,None])+beta*(src2))
    arr = np.zeros_like(dst)
    arr[:,:,0] = src1[:,:]
    arr[:,:,1] = src1[:,:]
    arr[:,:,2] = src1[:,:]
    img = np.concatenate((arr, dst), axis=0)
    #resize img
    scale_percent = 180 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return resized

def run_model(lidar):
    global mc
    global model
    global sess
    print("lidar.shape={}, mc.zenith={}, mc.azimuth={}".format(
        lidar.shape, mc.ZENITH_LEVEL, mc.AZIMUTH_LEVEL))
    lidar_mask = np.reshape(
        (lidar[:, :, 4] > 0),
        [mc.ZENITH_LEVEL, mc.AZIMUTH_LEVEL, 1]
    )
    lidar = (lidar - mc.INPUT_MEAN)/mc.INPUT_STD
    lidar = np.append(lidar, lidar_mask, axis=2)
    pred_cls = sess.run(
        model.pred_cls,
        feed_dict={
            model.lidar_input: [lidar],
            model.keep_prob: 1.0,
            model.lidar_mask: [lidar_mask]
        }
    )
    return pred_cls

def squeeze_seg_process_point_cloud(lidar, timestamp):
    
#     pred_cls = run_model(lidar)
#     resized = generate_lidar_images(lidar,pred_cls)
#     lidar_save = Image.fromarray(resized).convert('RGBA')
#     lidar_save.save(os.path.join(os.getenv("CARMEN_HOME") + '/sharedlib/libsqueeze_seg_v2/data/samples_out/', str(timestamp.item(0)) + '_full' + '.png'))
# 
#     cv2.imshow("Slices", resized)
#     cv2.waitKey(100)
#         
#     return pred_cls[0]
    
    #make copies for test
    lidar1 = lidar[:,255:767,:]
    lidarp1 = lidar[:,767:,:]
    lidarp2 = lidar[:,:255,:]
    lidar2 = np.hstack((lidarp1, lidarp2))
     
    pred_cls_lidar1 = run_model(lidar1)
    pred_cls_lidar2 = run_model(lidar2)
     
    img_lidar1 = generate_lidar_images(lidar1,pred_cls_lidar1)
    img_lidar2 = generate_lidar_images(lidar2,pred_cls_lidar2)
     
    img_to_test = np.concatenate((img_lidar1, img_lidar2), axis=0)
     
    cv2.imshow("Slices", img_to_test)
    cv2.waitKey(100)
    lidar_save = Image.fromarray(img_to_test).convert('RGBA')
    lidar_save.save(os.path.join(os.getenv("CARMEN_HOME") + '/sharedlib/libsqueeze_seg_v2/data/samples_out/', str(timestamp.item(0)) + '_slices' + '.png'))
    #print(pred_cls_lidar2[0].shape)
    pred_cls = np.hstack((pred_cls_lidar2[0][:,255:],pred_cls_lidar1[0], pred_cls_lidar2[0][:,:255]))
    print("pred_cls.shape={}".format(
        pred_cls.shape))
     
    return pred_cls

