from __future__ import absolute_import
from __future__ import division
# gtype import
from __future__ import print_function

#Activating virtualenv
import os

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

import det_model_fn
import hparams_config
import inference
import utils


global sess
global model_name
global ckpt_path
global image_size

model_name = os.getenv("MODEL")
ckpt_path = os.getenv("CKPT_PATH")
test_dir = carmen_home + "/sharedlib/efficientdet/testdata"

flags.DEFINE_string('logdir', '/tmp/deff/', 'log directory.')
flags.DEFINE_string('trace_filename', None, 'Trace file name.')
flags.DEFINE_integer('num_classes', 90, 'Number of classes.')
flags.DEFINE_integer('input_image_size', None, 'Size of input image.')
flags.DEFINE_integer('threads', 0, 'Number of threads.')
flags.DEFINE_integer('bm_runs', 20, 'Number of benchmark runs.')
flags.DEFINE_string('tensorrt', None, 'TensorRT mode: {None, FP32, FP16, INT8}')
flags.DEFINE_bool('delete_logdir', True, 'Whether to delete logdir.')
flags.DEFINE_bool('freeze', False, 'Freeze graph.')
flags.DEFINE_bool('xla', False, 'Run with xla optimization.')

flags.DEFINE_string('export_ckpt', None, 'Path for exporting new models.')
flags.DEFINE_bool('enable_ema', True, 'Use ema variables for eval.')

flags.DEFINE_string('input_image', None, 'Input image path for inference.')
flags.DEFINE_string('output_image_dir', '/tmp/', 'Output dir for inference.')

# For visualization.
flags.DEFINE_integer('line_thickness', None, 'Line thickness for box.')
flags.DEFINE_integer('max_boxes_to_draw', None, 'Max number of boxes to draw.')
flags.DEFINE_float('min_score_thresh', None, 'Score threshold to show box.')

FLAGS = flags.FLAGS


def initialize(width, height):
    # print ("ok, entrou no run_inplace_abn")
    global sess
    global model_name
    global ckpt_path
    global image_size
    
    sess = tf.Session(config=tf.ConfigProto(allow_soft_placement=True))

    model_overrides = {}
    model_params = hparams_config.get_detection_config(model_name)
    logdir = FLAGS.logdir
    tensorrt = FLAGS.tensorrt
    use_xla = FLAGS.xla
    enable_ema = FLAGS.enable_ema
    export_ckpt = FLAGS.export_ckpt
    image_size = FLAGS.image_size

    if image_size:
      # Use user specified image size.
      model_overrides = {'image_size': image_size}
    else:
      # Use default size.
      image_size = hparams_config.get_detection_config(model_name).image_size
      model_overrides = {}

    # A few fixed parameters.
    batch_size = 1
    inputs_shape = [batch_size, image_size, image_size, 3]
    labels_shape = [batch_size, FLAGS.num_classes]
    #image_size = width

    config_dict = {}
    if FLAGS.line_thickness:
        config_dict['line_thickness'] = FLAGS.line_thickness
    if FLAGS.max_boxes_to_draw:
        config_dict['max_boxes_to_draw'] = FLAGS.max_boxes_to_draw
    if FLAGS.min_score_thresh:
        config_dict['min_score_thresh'] = FLAGS.min_score_thresh
    driver = inference.InferenceDriver(model_name, ckpt_path,
                                       image_size)
    
    print("\n\n-------------------------------------------------------")
    print("       Pretrained Model EfficientDet loaded!")
    print("-------------------------------------------------------\n\n")


def efficientdet_process_image(carmen_image, timestamp):
    global sess
    global model_name
    global ckpt_path
    global image_size
    # converter a imagem
    print ("opaaaa!! entrou no efficientdet_process_image")
    image = Image.fromarray(carmen_image)

    # Buid inputs and preprocessing.
    raw_images, images, scales = [], [], []
    raw_images.append(image)
    image, scale = image_preprocess(image, image_size)
    images.append(image)
    scales.append(scale)
  
    # Build model.
    class_outputs, box_outputs = build_model(
          self.model_name, images, **self.params)
    restore_ckpt(sess, self.ckpt_path, enable_ema=True, export_ckpt=None)
    params.update(dict(batch_size=1))  # required by postprocessing.

      # Build postprocessing.
    detections_batch = det_post_process(
          params, class_outputs, box_outputs, scales)
    outputs_np = sess.run(detections_batch)
    output_np = outputs_np[0]
    
    # output_np has format [image_id, x, y, width, height, score, class]
    boxes = output_np[:, 1:5]
    classes = output_np[:, 6].astype(int)
    scores = output_np[:, 5]
    # convert [x, y, width, height] to [ymin, xmin, ymax, xmax]
    # TODO(tanmingxing): make this convertion more efficient.
    boxes[:, [0, 1, 2, 3]] = boxes[:, [1, 0, 3, 2]]
    boxes[:, 2:4] += boxes[:, 0:2]
    img = visualize_image(raw_images[i], boxes, classes, scores,
                              self.label_id_mapping, **kwargs)
    output_image_path = os.path.join(test_dir, str(timestamp.item(0)) + '.jpg')
    Image.fromarray(img).save(output_image_path)
    tf.logging.info('writing file to {}'.format(output_image_path))

    return 


