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

def save_txt(data, file_name):
  
  # Write the array to disk
  with open(os.path.join(os.getenv("CARMEN_HOME") + '/sharedlib/libsqueeze_seg_v2/data/samples_IARA/', file_name+'.txt'), 'w') as outfile:
    # I'm writing a header here just for the sake of readability
    # Any line starting with "#" will be ignored by numpy.loadtxt
    outfile.write('# Array shape: {0}\n'.format(data.shape))

    # Iterating through a ndimensional array produces slices along
    # the last axis. This is equivalent to data[i,:,:] in this case
    for data_slice in data:

        # The formatting string indicates that I'm writing out
        # the values in left-justified columns 7 characters in width
        # with 2 decimal places.  
        np.savetxt(outfile, data_slice, fmt='%-7.2f', delimiter='\t')

        # Writing out a break to indicate different slices...
        outfile.write('# New slice\n')
        
def save_lidar_image(img_file, timestamp):
    lidar_save = Image.fromarray(img_file).convert('RGBA')
    lidar_save.save(os.path.join(os.getenv("CARMEN_HOME") + '/sharedlib/libsqueeze_seg_v2/data/samples_out/', str(timestamp.item(0)) + '_slices' + '.png'))

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
    #print("lidar.shape={}, mc.zenith={}, mc.azimuth={}".format(
    #    lidar.shape, mc.ZENITH_LEVEL, mc.AZIMUTH_LEVEL))
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
    #save_txt(lidar, str(timestamp.item(0)))
    lidar1 = lidar[:,271:783,:]
    lidarp1 = lidar[:,783:,:]
    shape_last_part = lidar.shape[1] - 783 #300
    shape_to_complete = mc.AZIMUTH_LEVEL - shape_last_part #212
    shape_to_squeeze = shape_to_complete + mc.AZIMUTH_LEVEL
    #print ("shape_last_part=" + str(shape_last_part) + " shape_complete=" + str(shape_to_complete) + " shape_to_squeeze=" + str(shape_to_squeeze))
    lidarp2 = lidar[:,:shape_to_complete,:]
    lidar2 = np.hstack((lidarp1, lidarp2))
    '''Has to do something between 212 to 271, so do lidar3'''
    lidar3 = lidar[:,shape_to_complete:shape_to_squeeze,:]
    
    pred_cls_lidar1 = run_model(lidar1)
    pred_cls_lidar2 = run_model(lidar2)
    pred_cls_lidar3 = run_model(lidar3)
     
    img_lidar1 = generate_lidar_images(lidar1,pred_cls_lidar1)
    img_lidar2 = generate_lidar_images(lidar2,pred_cls_lidar2)
    #img_lidar3 = generate_lidar_images(lidar3,pred_cls_lidar3)
     
    img_to_test = np.concatenate((img_lidar1, img_lidar2), axis=0)
     
    cv2.imshow("Slices", img_to_test)
    cv2.waitKey(100)
    #save_lidar_image(img_to_test, timestamp)
    
    pred_cls = np.hstack((pred_cls_lidar2[0][:,shape_to_complete:],pred_cls_lidar3[0][:,shape_to_complete:271],pred_cls_lidar1[0], pred_cls_lidar2[0][:,:shape_to_complete]))
    print("pred_cls.shape={}".format(
        pred_cls.shape))
     
    return pred_cls

