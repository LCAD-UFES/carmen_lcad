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


import tensorflow.compat.v1 as tf
import numpy as np
from PIL import Image, ImagePalette
#import cv2
from typing import Text, Dict, Any, List

import anchors
import dataloader
import det_model_fn
import hparams_config
import utils
#import model_inference
#import model_inspect
import inference
from visualize import vis_utils

os.environ['CUDA_VISIBLE_DEVICES'] = '0'

global model_name
global ckpt_path
global driver

model_name = 'efficientdet-d0'
ckpt_path = carmen_home + "/sharedlib/efficientdet/efficientdet-d0"
test_dir = carmen_home + "/sharedlib/efficientdet/testdata"
image_size = None

def initialize():
    global model_name
    global ckpt_path
    global driver
    tf.reset_default_graph()
    tf.compat.v1.disable_eager_execution()
    driver = inference.ServingDriver(model_name, ckpt_path)
    driver.build()
    print("\n\n-------------------------------------------------------")
    print("       EfficientDet loaded!")
    print("-------------------------------------------------------\n\n")

def efficientdet_process_image(carmen_image):
    global model_name
    global ckpt_path
    global driver
    # converter a imagem
    image = Image.fromarray(carmen_image)
    predictions = driver.serve(image)
    predret = np.array(predictions[0][:,1:7], dtype=np.float)
    return predret
