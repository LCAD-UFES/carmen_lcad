import os
import cv2
import numpy as np
from collections import OrderedDict

import torch
import torch.backends.cudnn as cudnn

from PIL import Image
from torchvision import transforms

import utils.logging as logging
import utils.metrics as metrics
from models.model import GLPDepth

def activate_virtual_environment(environment_root):
    """Configures the virtual environment starting at ``environment_root``."""
    activate_script = os.path.join(
        environment_root, 'bin', 'activate_this.py')
    exec(compile(open(activate_script, "rb").read(), activate_script, 'exec'), dict(__file__=activate_script))

carmen_home = os.getenv("CARMEN_HOME")
virtualenv_root = carmen_home + "/src/deep_mapper/GLPDepth/venv"
activate_virtual_environment(virtualenv_root)


global model
global device

def initialize():
    global model
    global device
    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("GLPDepth: device: %s" % device)
    print("GLPDepth: Define Model with kitti")
    model = GLPDepth(max_depth=80.0, is_train=False).to(device)
    model_weight = torch.load(carmen_home + '/src/deep_mapper/GLPDepth/ckpt/best_model_kitti.ckpt')
    if 'module' in next(iter(model_weight.items()))[0]:
        model_weight = OrderedDict((k[7:], v) for k, v in model_weight.items())
    model.load_state_dict(model_weight)
    model.eval()
    if device == torch.device("cuda"):
        model = model.to(memory_format=torch.channels_last)
        model = model.half()

    model.to(device)

    print("\n\n-------------------------------------------------------")
    print("       Pretrained Model GLPDepth loaded!")
    print("-------------------------------------------------------\n\n")


def glp_process_image(image):
    global model
    global device
    
    print("GLPDepth: Inference & Evaluate")

    img = image
    img = img[:, :, ::-1].copy() 
    # print(img.shape)
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) / 255.0

    with torch.no_grad():
        tensor_img = torch.Tensor(img)
        input_RGB = tensor_img.to(device)
        pred = model(input_RGB)
        pred_d = pred['pred_d']
        pred_d = pred_d.squeeze()
        pred_d = pred_d.cpu().numpy() * 256.0
    return (pred_d).astype('uint16')
            
        