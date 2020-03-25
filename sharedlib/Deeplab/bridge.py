import os

def activate_virtual_environment(environment_root):
    """Configures the virtual environment starting at ``environment_root``."""
    activate_script = os.path.join(
        environment_root, 'bin', 'activate_this.py')
    execfile(activate_script, {'__file__': activate_script})

carmen_home = os.getenv("CARMEN_HOME")
virtualenv_root = carmen_home + "/sharedlib/Deeplab/deeplab_env"
activate_virtual_environment(virtualenv_root)

import numpy as np
from PIL import Image

# Imports just from my module
from model import DeepLabModel
from visualize import vis_segmentation, label_to_color_image

os.environ['CUDA_VISIBLE_DEVICES'] =''

saved_path = carmen_home + '/sharedlib/Deeplab/deeplab_model_cityscapes.tar.gz'
model = DeepLabModel(saved_path)

def process_image(carmen_image):
    img_teste = Image.fromarray(carmen_image)
    resized_im, seg_map = model.run(img_teste)
    seg_map = seg_map.astype(np.uint8)
    return seg_map

def color_image(seg_map):
    seg_image = label_to_color_image(seg_map).astype(np.uint8)
    return seg_image

def process_image_from_disk(png_filename):
    img_teste = Image.open(png_filename)
    resized_im, seg_map = model.run(img_teste)
    vis_segmentation(resized_im, seg_map)

if __name__ == "__main__":
    from sys import argv, version_info
    if len(argv) < 2 or argv[1] == '-h':
        print('\nUsage: python' + str(version_info.major) + ' ' + argv[0] + ' <png_filenames> ...\n')
        exit()
    for filename in argv[1:]:
        process_image_from_disk(filename)
