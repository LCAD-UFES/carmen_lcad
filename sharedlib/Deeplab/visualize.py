import matplotlib.pyplot as plt
from matplotlib import gridspec

import numpy as np
# from PIL import Image

def vis_segmentation(image, seg_map):
  """Visualizes input image, segmentation map and overlay view."""
  plt.figure(figsize=(45, 15))
  grid_spec = gridspec.GridSpec(1, 4, width_ratios=[6, 6, 6, 1])
  
  plt.subplot(grid_spec[0])
  plt.imshow(image)
  plt.axis('off')
  plt.title('input image')

  plt.subplot(grid_spec[1])
  seg_image = label_to_color_image(seg_map).astype(np.uint8)
  plt.imshow(seg_image)
  plt.axis('off')
  plt.title('segmentation map')

  plt.subplot(grid_spec[2])
  plt.imshow(image, extent=[0, seg_image.shape[1], seg_image.shape[0], 0])
  plt.imshow(seg_image, alpha=0.7)
  plt.axis('off')
  plt.title('segmentation overlay')

  unique_labels = np.unique(seg_map)
  ax = plt.subplot(grid_spec[3])
  plt.imshow(
      FULL_COLOR_MAP[unique_labels].astype(np.uint8), interpolation='nearest')
  ax.yaxis.tick_right()
  plt.yticks(range(len(unique_labels)), LABEL_NAMES[unique_labels])
  plt.xticks([], [])
  ax.tick_params(width=0.0)
  plt.grid('off')
  plt.show()

def create_cityscapes_label_colormap():
  """Creates a label colormap used in CITYSCAPES segmentation benchmark.

  Returns:
    A Colormap for visualizing segmentation results.
  """
  colormap = np.asarray([
      [128, 64, 128],
      [244, 35, 232],
      [70, 70, 70],
      [102, 102, 156],
      [190, 153, 153],
      [153, 153, 153],
      [250, 170, 30],
      [220, 220, 0],
      [107, 142, 35],
      [152, 251, 152],
      [70, 130, 180],
      [220, 20, 60],
      [255, 0, 0],
      [0, 0, 142],
      [0, 0, 70],
      [0, 60, 100],
      [0, 80, 100],
      [0, 0, 230],
      [119, 11, 32],
  ])
  return colormap


def label_to_color_image(label):
  """Adds color defined by the dataset colormap to the label.

  Args:
    label: A 2D array with integer type, storing the segmentation label.

  Returns:
    result: A 2D array with floating type. The element of the array
      is the color indexed by the corresponding element in the input label
      to the PASCAL color map.

  Raises:
    ValueError: If label is not of rank 2 or its value is larger than color
      map maximum entry.
  """
  if label.ndim != 2:
    raise ValueError('Expect 2-D input label')

  colormap = create_cityscapes_label_colormap()
  
  if np.max(label) >= len(colormap):
    raise ValueError('label value too large.')

  return colormap[label]

#Labels para modelo *_cityscapes_*
LABEL_NAMES = np.asarray([
    'road', 
    'sidewalk', 
    'building', 
    'wall',
    'fence',
    'pole',
    'trafficlight',
    'trafficsign',
    'vegetation',
    'terrain',
    'sky',
    'person',
    'rider',
    'car',
    'truck',
    'bus',
    'train',
    'motorcycle',
    'bicycle'
])

FULL_LABEL_MAP = np.arange(len(LABEL_NAMES)).reshape(len(LABEL_NAMES), 1)
FULL_COLOR_MAP = label_to_color_image(FULL_LABEL_MAP)

def get_label_name_by_number(label_number):
    if 0 <= label_number < len(LABEL_NAMES):
        return LABEL_NAMES[label_number]
    return ''

MOVING_OBJECTS = []
for label_number in range(len(LABEL_NAMES)):
    if LABEL_NAMES[label_number] in ('person','rider','car','truck','bus','train','motorcycle','bicycle'):
        MOVING_OBJECTS.append(label_number)
MOVING_OBJECTS = np.asarray(MOVING_OBJECTS)

def is_moving_object(label_number):
    return int(label_number in MOVING_OBJECTS)
