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
from visualize import vis_segmentation

saved_path = carmen_home + '/sharedlib/Deeplab/deeplab_model_cityscapes.tar.gz'
model = DeepLabModel(saved_path)

def process_image(carmen_image):
  img_teste = Image.fromarray(carmen_image)
  
  
  resized_im, seg_map = model.run(img_teste)
  seg_map = seg_map.astype(np.uint8)

  # vis_segmentation(resized_im, seg_map)

  return seg_map



#Para rodar de forma standalone. Troque o nome do arquivo teste.png para aquele que vc usar de teste
# def process_image_from_disk():
#   img_teste = Image.open('teste.png')
#   resized_im, seg_map = model.run(img_teste)
#   vis_segmentation(resized_im, seg_map)
# process_image_from_disk()
