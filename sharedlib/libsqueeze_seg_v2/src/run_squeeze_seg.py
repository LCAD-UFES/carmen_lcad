from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from datetime import datetime

import os
import os.path
import sys
import time
import glob
#import cv2

import numpy as np
from tabulate import tabulate
from six.moves import xrange
import tensorflow as tf
from PIL import Image

from config import *
from imdb import kitti
from utils.util import *
from nets import *
#sys.argv = sys.argv[:1]
carmen_home = os.getenv("CARMEN_HOME")
base_path = carmen_home + '/sharedlib/libsqueeze_seg_v2/'
"""Loads pretrained SqueezeSegV2 model."""
mc = kitti_squeezeSeg_config()
mc.LOAD_PRETRAINED_MODEL = False
mc.BATCH_SIZE = 1
model = SqueezeSeg(mc)
graph = tf.Graph().as_default()
sess = tf.Session(config=tf.ConfigProto(allow_soft_placement=True))
saver = tf.train.Saver(model.model_params)
saver.restore(sess, base_path +
                   'src/model/SqueezeSegV2/model.ckpt-30700')
print("Pretrained model Loaded!")

def _normalize(x):
    return (x - x.min())/(x.max() - x.min())


def squeeze_seg_process_point_cloud(lidar, timestamp):
    """Detect LiDAR data."""
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
    depth_map = Image.fromarray(
        (255 * _normalize(lidar[:, :, 3])).astype(np.uint8))
    #cv2.imshow('depth_map', depth_map)
    depth_map.save(
        os.path.join(base_path + 'data/samples_out/', 'in_' + str(timestamp.item(0)) + '.png'))
    label_map = Image.fromarray(
        (255 * visualize_seg(pred_cls, mc)[0]).astype(np.uint8))
    blend_map = Image.blend(
        depth_map.convert('RGBA'),
        label_map.convert('RGBA'),
        alpha=0.4
    )
    blend_map.save(
        os.path.join(base_path + 'data/samples_out/', 'out_' + str(timestamp.item(0)) + '.png'))
    #cv2.imshow('blend_map', blend_map)
    return pred_cls[0]


