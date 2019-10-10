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
    mc.AZIMUTH_LEVEL = shots_to_squeeze

    mc.LOAD_PRETRAINED_MODEL = False
    mc.BATCH_SIZE = 1
    model = SqueezeSeg(mc)

    '''Loads tensorflow'''
    graph = tf.Graph().as_default()
    sess = tf.Session(config=tf.ConfigProto(allow_soft_placement=True))
    saver = tf.train.Saver(model.model_params)
    saver.restore(sess, os.getenv("CARMEN_HOME") + '/sharedlib/libsqueeze_seg_v2/data/SqueezeSegV2/model.ckpt-30700')

    print ("\n\n-------------------------------------------------------")
    print ("       Pretrained Model and Tensorflow loaded!")
    print ("-------------------------------------------------------\n\n")


def _normalize(x):
    return (x - x.min())/(x.max() - x.min())


def squeeze_seg_process_point_cloud(lidar, timestamp):
    global mc
    global model
    global sess

    #print("timestamp={}".format(timestamp.item(0)))
    #print("lidar.shape={}, mc.zenith={}, mc.azimuth={}".format(
    #    lidar.shape, mc.ZENITH_LEVEL, mc.AZIMUTH_LEVEL))

    lidar_mask = np.reshape(
        (lidar[:, :, 4] > 0),
        [mc.ZENITH_LEVEL, mc.AZIMUTH_LEVEL, 1]
    )
    #print("lidar_mask.shape={}".format(lidar_mask.shape))
    lidar = (lidar - mc.INPUT_MEAN)/mc.INPUT_STD
    lidar = np.append(lidar, lidar_mask, axis=2)
    #print('Before predict')
    pred_cls = sess.run(
        model.pred_cls,
        feed_dict={
            model.lidar_input: [lidar],
            model.keep_prob: 1.0,
            model.lidar_mask: [lidar_mask]
        }
    )
    #print('After predict')
    depth_map = Image.fromarray((255 * _normalize(lidar[:, :, 3])).astype(np.uint8))
    #depth_map.save(os.path.join(os.getenv("CARMEN_HOME") + '/sharedlib/libsqueeze_seg_v2/data/samples_out/', 'depth_map_' + str(timestamp.item(0)) + '.png'))
    label_map = Image.fromarray((255 * visualize_seg(pred_cls, mc)[0]).astype(np.uint8))
    blend_map = Image.blend(depth_map.convert('RGBA'), label_map.convert('RGBA'), alpha=0.4)
    #blend_map.save(os.path.join(os.getenv("CARMEN_HOME") + '/sharedlib/libsqueeze_seg_v2/data/samples_out/', 'blend_map' + str(timestamp.item(0)) + '.png'))
    alpha = 0.5
    beta = (1.0 - alpha)
    src1 = (255 * _normalize(lidar[:, :, 3])).astype(np.uint8)
    src2 = (255 * visualize_seg(pred_cls, mc)[0]).astype(np.uint8)
    dst = np.uint8(alpha*(src1[:,:,None])+beta*(src2))
    #dst = cv2.addWeighted(src1[:,:,None], alpha, src2, beta, 0.0)
    cv2.imshow("Blend Map", dst)
    cv2.waitKey(50)
    
    #print(len(pred_cls[0])) #= 32
    #print(len(pred_cls[0][0])) #= 1024
    #print(type(pred_cls[0][0][0])) int64
    
    return pred_cls[0]


