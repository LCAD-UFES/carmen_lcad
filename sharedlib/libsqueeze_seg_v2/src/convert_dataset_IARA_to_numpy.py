# Author: Bichen Wu (bichen@berkeley.edu) 03/07/2017

"""Evaluation"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from datetime import datetime

import os

def activate_virtual_environment(environment_root):
    """Configures the virtual environment starting at ``environment_root``."""
    activate_script = os.path.join(
        environment_root, 'bin', 'activate_this.py')
    execfile(activate_script, {'__file__': activate_script})

carmen_home = os.getenv("CARMEN_HOME")
virtualenv_root = carmen_home + "/sharedlib/libsqueeze_seg_v2/squeezeseg_env"
activate_virtual_environment(virtualenv_root)

import os.path
import sys
import time
import glob    

import numpy as np
from six.moves import xrange
import tensorflow as tf

FLAGS = tf.app.flags.FLAGS

tf.app.flags.DEFINE_string(
        'input_path', './data/train/*',
        """Input lidar scan to be detected. Can process glob input such as """
        """./data/samples/*.npy or single input.""")
tf.app.flags.DEFINE_string(
        'out_dir', './data/train_npy/', """Directory to dump output.""")
tf.app.flags.DEFINE_string('gpu', '0', """gpu id.""")

def vertical_doubled(lidar):
    s = (lidar.shape[0]*2, lidar.shape[1], lidar.shape[2])
    lidar_new = np.zeros(s, dtype=np.float32)
    i = 0
    for x in range(lidar.shape[0]-1):
        first_set = lidar[x,:,:]
        second_set = lidar[x+1,:,:]
        lidar_new[i] = first_set
        lidar_new[i+1] = first_set
        lidar_new[i+2] = second_set
        i = i + 2
    return lidar_new

def txt_to_npy():
    """Transform LiDAR data."""
    for f in glob.iglob(FLAGS.input_path):
        raw_lidar = np.loadtxt(f, delimiter = '\t')
        #print (raw_lidar.shape)
        vertical_resolution = 32
        shots = raw_lidar.shape[0]//vertical_resolution
        raw_lidar = raw_lidar.reshape((vertical_resolution, shots, 6))
        lidar = vertical_doubled(raw_lidar)
        lidar_part = lidar[:,288:800,:]
        # print(lidar_part.shape)
        file_name = f.strip('.txt').split('/')[-1]
        #np.save(
        #    os.path.join(FLAGS.out_dir, 'original_'+file_name+'.npy'),
        #    lidar_part
        #)
        label = lidar_part[:,:,5]
        # print (label.shape)
        result = np.where(label[:]>3.0, 0.0, label[:])
        lidar_part[:,:,5] = result
        # print (lidar_part.shape)
        file_name = f.strip('.txt').split('/')[-1]
        np.save(
            os.path.join(FLAGS.out_dir, 'iara_'+file_name+'.npy'),
            lidar_part
        )

def main(argv=None):
  if not tf.gfile.Exists(FLAGS.out_dir):
    tf.gfile.MakeDirs(FLAGS.out_dir)
  txt_to_npy()
  print('Output written to {}'.format(FLAGS.out_dir))


if __name__ == '__main__':
    tf.app.run()
