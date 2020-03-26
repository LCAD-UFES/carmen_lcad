from __future__ import absolute_import
from __future__ import division
# gtype import
from __future__ import print_function

#Activating virtualenv
import os
import sys

def activate_virtual_environment(environment_root):
    """Configures the virtual environment starting at ``environment_root``."""
    activate_script = os.path.join(
        environment_root, 'bin', 'activate_this.py')
    exec(compile(open(activate_script, "rb").read(), activate_script, 'exec'), dict(__file__=activate_script))

carmen_home = os.getenv("CARMEN_HOME")
virtualenv_root = carmen_home + "/sharedlib/efficientdet/venv"
activate_virtual_environment(virtualenv_root)
#virtualenv activated
import time

from absl import logging
from absl import flags
import numpy as np
from PIL import Image, ImagePalette
import tensorflow.compat.v1 as tf
import cv2
from typing import Text, Dict, Any, List

import anchors
import dataloader
import det_model_fn
import hparams_config
import utils
#import model_inference
import model_inspect
import inference_iara
from visualize import vis_utils

os.environ['CUDA_VISIBLE_DEVICES'] = '0'

global sess
global model_name
global ckpt_path
global inspector
global image_size

model_name = 'efficientdet-d0'
ckpt_path = 'efficientdet-d0'
test_dir = carmen_home + "/sharedlib/efficientdet/testdata"
image_size = None

def initialize(width, height):
    # print ("ok, entrou no run_inplace_abn")
    global sess
    global model_name
    global ckpt_path
    global inspector
    global image_size
    
    logdir = '/tmp/deff/'
    delete_logdir = True
    num_classes = 90
    tensorrt = None
    enable_ema = True
    #threads = 0
    sess = tf.Session(config=tf.ConfigProto(allow_soft_placement=True))

    #init
    logging.set_verbosity(logging.WARNING)
    tf.logging.set_verbosity(tf.logging.WARN)
    tf.disable_v2_behavior()
    if tf.io.gfile.exists(logdir) and delete_logdir:
      tf.logging.info('Deleting log dir ...')
      tf.io.gfile.rmtree(logdir)
    print (model_name)
    inspector = model_inspect.ModelInspector(
      model_name=model_name,
      image_size=image_size,
      num_classes=num_classes,
      logdir=logdir,
      tensorrt=tensorrt,
      use_xla=False,
      ckpt_path=ckpt_path,
      enable_ema=enable_ema,
      export_ckpt=None)
    #inspector.run_model(threads)
    
    print("\n\n-------------------------------------------------------")
    print("       Pretrained Model EfficientDet loaded!")
    print("-------------------------------------------------------\n\n")

def efficientdet_process_image(carmen_image, timestamp):
    global sess
    global model_name
    global ckpt_path
    global image_size
    global params

    # converter a imagem
    print ("opaaaa!! entrou no efficientdet_process_image")
    image = Image.fromarray(carmen_image)
    image.show()
    
    driver = inference_iara.InferenceDriver(model_name, ckpt_path,
                                       image_size)
    driver.inference_image(image, test_dir, timestamp, **kwargs)
    return [0, None]
    ## Buid inputs and preprocessing.
    #raw_images, images, scales = [], [], []
    #raw_images.append(image)
    #image, scale = image_preprocess(image, 640)
    #images.append(image)
    #scales.append(scale)
  #
    ## Build model.
    #class_outputs, box_outputs = build_model(
    #      model_name, images, **params)
    #restore_ckpt(sess, ckpt_path, enable_ema=True, export_ckpt=None)
    #params.update(dict(batch_size=1))  # required by postprocessing.
#
    #  # Build postprocessing.
    #detections_batch = det_post_process(
    #      params, class_outputs, box_outputs, scales)
    #outputs_np = sess.run(detections_batch)
    #output_np = outputs_np[0]
    #
    ## output_np has format [image_id, x, y, width, height, score, class]
    #boxes = output_np[:, 1:5]
    #classes = output_np[:, 6].astype(int)
    #scores = output_np[:, 5]
    ## convert [x, y, width, height] to [ymin, xmin, ymax, xmax]
    ## TODO(tanmingxing): make this convertion more efficient.
    #boxes[:, [0, 1, 2, 3]] = boxes[:, [1, 0, 3, 2]]
    #boxes[:, 2:4] += boxes[:, 0:2]
    #img = visualize_image(raw_images[i], boxes, classes, scores,
    #                          label_id_mapping, **kwargs)
    #output_image_path = os.path.join(test_dir, str(timestamp.item(0)) + '.jpg')
    #Image.fromarray(img).save(output_image_path)
    #tf.logging.info('writing file to {}'.format(output_image_path))
#
    #return [boxes.size(0), boxes[:,:]]
#
