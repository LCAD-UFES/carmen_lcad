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
from PIL import Image

from config import *
from imdb import kitti
from utils.util import *
from nets import *

FLAGS = tf.app.flags.FLAGS

tf.app.flags.DEFINE_string(
        'checkpoint', './data/SqueezeSegV2/model.ckpt-30700',
        """Path to the model parameter file.""")
tf.app.flags.DEFINE_string(
        'input_path', './data/train/*',
        """Input lidar scan to be detected. Can process glob input such as """
        """./data/samples/*.npy or single input.""")
tf.app.flags.DEFINE_string(
        'out_dir', './data/samples_out/', """Directory to dump output.""")
tf.app.flags.DEFINE_string('gpu', '0', """gpu id.""")

def _normalize(x):
  return (x - x.min())/(x.max() - x.min())

def save_txt(data, f):
  file_name = f.strip('.npy').split('/')[-1]
  
  # Write the array to disk
  with open(os.path.join(FLAGS.out_dir, 'entrada_'+file_name+'.txt'), 'w') as outfile:
    # I'm writing a header here just for the sake of readability
    # Any line starting with "#" will be ignored by numpy.loadtxt
    outfile.write('# Array shape: {0}\n'.format(data.shape))

    # Iterating through a ndimensional array produces slices along
    # the last axis. This is equivalent to data[i,:,:] in this case
    for data_slice in data:

        # The formatting string indicates that I'm writing out
        # the values in left-justified columns 7 characters in width
        # with 2 decimal places.  
        np.savetxt(outfile, data_slice, fmt='%-7.2f')

        # Writing out a break to indicate different slices...
        outfile.write('# New slice\n')

def get_concat_v(im1, im2):
    dst = Image.new('RGB', (im1.width, im1.height + im2.height))
    dst.paste(im1, (0, 0))
    dst.paste(im2, (0, im1.height))
    return dst

def detect():
  """Detect LiDAR data."""

  os.environ['CUDA_VISIBLE_DEVICES'] = FLAGS.gpu

  with tf.Graph().as_default():
    mc = kitti_squeezeSeg_config()
    mc.ZENITH_LEVEL = 32
    mc.AZIMUTH_LEVEL = 512
    mc.LOAD_PRETRAINED_MODEL = False
    mc.BATCH_SIZE = 1 # TODO(bichen): fix this hard-coded batch size.
    model = SqueezeSeg(mc)

    saver = tf.train.Saver(model.model_params)
    with tf.Session(config=tf.ConfigProto(allow_soft_placement=True)) as sess:
      saver.restore(sess, FLAGS.checkpoint)
      for f in glob.iglob(FLAGS.input_path):
        raw_lidar = np.loadtxt(f, delimiter = '\t')
        #print (raw_lidar.shape)
        vertical_resolution = 32
        shots = raw_lidar.shape[0]//vertical_resolution
        raw_lidar = raw_lidar.reshape((vertical_resolution, shots, 6))
        lidar = raw_lidar[:,288:800,:5]
        pred_test = raw_lidar[:,288:800,5]
        s = (1, pred_test.shape[0], pred_test.shape[1])
        pred_new = np.zeros(s, dtype=np.int64)
        pred_new[0] = pred_test
        print (lidar.shape)
        print (pred_new.shape)
        # lidar = np.load(f).astype(np.float32, copy=False)[:, :, :5]
        # save_txt(lidar, f)
        lidar_mask = np.reshape(
            (lidar[:, :, 4] > 0),
            [mc.ZENITH_LEVEL, mc.AZIMUTH_LEVEL, 1]
        )
        lidar = (lidar - mc.INPUT_MEAN)/mc.INPUT_STD
        lidar = np.append(lidar, lidar_mask, axis=2)
        pred_cls = sess.run(
            model.pred_cls,
            feed_dict={
                model.lidar_input:[lidar],
                model.keep_prob: 1.0,
                model.lidar_mask:[lidar_mask]
            }
        )
        print (pred_cls.shape)
        # save the data
        file_name = f.strip('.txt').split('/')[-1]
        
        # np.save(
        #     os.path.join(FLAGS.out_dir, 'pred_'+file_name+'.npy'),
        #     pred_cls[0]
        # )
        #np.savetxt('foo'+file_name+'.csv', pred_cls[0].shape, delimiter=",")

        # save the plot
        depth_map = Image.fromarray(
            (255 * _normalize(lidar[:, :, 3])).astype(np.uint8))
        #depth_map.save(
        #    os.path.join(FLAGS.out_dir, 'in_'+file_name+'.png'))

        label_map = Image.fromarray(
            (255 * visualize_seg(pred_cls, mc)[0]).astype(np.uint8))
        
        label_train_map = Image.fromarray(
            (255 * visualize_seg(pred_new, mc)[0]).astype(np.uint8))

        blend_map = Image.blend(
            depth_map.convert('RGBA'),
            label_map.convert('RGBA'),
            alpha=0.4
        )

        blend_train_map = Image.blend(
            depth_map.convert('RGBA'),
            label_train_map.convert('RGBA'),
            alpha=0.4
        )

        #blend_map.save(
        #    os.path.join(FLAGS.out_dir, 'out_'+file_name+'.png'))
        dst = get_concat_v(depth_map, blend_map)
        dst_train = get_concat_v(depth_map, blend_train_map)
        dst.save(
            os.path.join(FLAGS.out_dir, file_name+'_out.png'))
        dst_train.save(
            os.path.join(FLAGS.out_dir, file_name+'_out_train.png'))
        


def main(argv=None):
  if not tf.gfile.Exists(FLAGS.out_dir):
    tf.gfile.MakeDirs(FLAGS.out_dir)
  detect()
  print('Detection output written to {}'.format(FLAGS.out_dir))


if __name__ == '__main__':
    tf.app.run()
