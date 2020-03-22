from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from datetime import datetime

#Activating virtualenv
import os

def activate_virtual_environment(environment_root):
    """Configures the virtual environment starting at ``environment_root``."""
    activate_script = os.path.join(
        environment_root, 'bin', 'activate_this.py')
    execfile(activate_script, {'__file__': activate_script})

carmen_home = os.getenv("CARMEN_HOME")
virtualenv_root = carmen_home + "/sharedlib/libsqueeze_seg_v2/squeezeseg_env"
activate_virtual_environment(virtualenv_root)
#virtualenv activated

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

#os.environ['CUDA_VISIBLE_DEVICES'] =''

def save_txt(data, file_name, tag):
  # Write the array to disk
  with open(os.path.join(os.getenv("CARMEN_HOME") + '/sharedlib/libsqueeze_seg_v2/data/samples_IARA/', file_name+ tag +'.txt'), 'w') as outfile:
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
        
def save_lidar_image(img_file, timestamp, tag):
    lidar_save = Image.fromarray(img_file).convert('RGBA')
    lidar_save.save(os.path.join(os.getenv("CARMEN_HOME") + '/sharedlib/libsqueeze_seg_v2/data/samples_out/', str(timestamp.item(0)) + tag + '.png'))

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
    scale_percent = 200 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100) * 2
    dim = (width, height)
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return resized

def squeeze_seg_save_npy(lidar, timestamp):
    np.save(os.path.join(os.getenv("CARMEN_HOME") + '/sharedlib/libsqueeze_seg_v2/data/samples/', str(timestamp.item(0))), lidar)
    return 0


