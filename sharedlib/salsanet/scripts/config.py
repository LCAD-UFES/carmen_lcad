import numpy as np
from easydict import EasyDict as edict
 
def lidar_config():

  cfg = edict()

  """road-vehicle segmentation using 3D LIDAR point clouds"""
  # classes
  cfg.CLASSES = [
      'background',
      'road',
      'vehicle']

  cfg.NUM_CLASS = len(cfg.CLASSES)    # number of classes
  cfg.CLS_2_ID = dict(zip(cfg.CLASSES, range(len(cfg.CLASSES))))      # dict from class name to id

  # rgb color for each class
  cfg.CLS_COLOR_MAP = np.array(
      [[ 255.00,  255.00,  255.00],
       [ 0.00,  255.00, 0.00],
       [ 255.00, 0.00, 0.00]])

  cfg.CLS_LOSS_WEIGHTS = np.array([1.01,  6.03, 15.78]) # smooth_freq weight values

  cfg.GPU = 0                    # gpu id
  cfg.DROPOUT_PROB = 0.5         # Probability to keep a node in dropout
  cfg.NUM_EPOCHS = 300           # epoch number
  cfg.BATCH_SIZE = 2             # batch size
  cfg.LEARNING_RATE = 0.01       # learning rate
  cfg.LR_DECAY_FACTOR = 0.1      # multiply the learning rate by this factor
  cfg.LR_DECAY_CYCLE = 20000     # step time to decrease the learning rate
  cfg.PRINT_EVERY = 20           # print in every 20 epochs
  cfg.DEBUG_MODE = False         # print log to console in debug mode
  cfg.DATA_AUGMENTATION = True   # Whether to do data augmentation
  cfg.CHANNEL_LABELS = [ 'mean',  'max', 'ref',  'den']   #channel names
  cfg.IMAGE_WIDTH = 256          # image width
  cfg.IMAGE_HEIGHT = 32          # image height
  cfg.IMAGE_CHANNEL = len(cfg.CHANNEL_LABELS) # image channel

  # paths
  cfg.training_data_path = "../data/salsaNet_bev/salsaNet_bev_train/"
  cfg.validation_data_path = "../data/salsaNet_bev/salsaNet_bev_val/"
  cfg.log_path = "../logs/"
  cfg.log_name = ""   # additional descriptive tag name to the log file if needed

  return cfg
