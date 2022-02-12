import os
import cv2
import numpy as np
from collections import OrderedDict

import torch
import torch.backends.cudnn as cudnn

from PIL import Image

import utils.logging as logging
import utils.metrics as metrics
from models.model import GLPDepth
import torchvision.transforms as transforms
import albumentations as A

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

    print("\n\n-------------------------------------------------------")
    print("       Pretrained Model GLPDepth loaded!")
    print("-------------------------------------------------------\n\n")


def glp_process_image(image, cut, down_cut):
    global model
    global device
    
    # print("GLPDepth: Inference & Evaluate")
    to_tensor = transforms.ToTensor()
    batch = {}

    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # input size should be multiple of 32
    h, w, c = image.shape
    new_h, new_w = h // 32 * 32, w // 32 * 32
    image = cv2.resize(image, (new_w, new_h))
    image = to_tensor(image)
    image = image.unsqueeze(0)
    
    batch['image'] = image
    input_RGB = batch['image'].to(device)

    with torch.no_grad():
        pred = model(input_RGB)
    pred_d = pred['pred_d']
    pred_d = pred_d.squeeze()
    pred_d = pred_d.cpu().numpy() * 256.0
    #print(pred_d)
    pred_d_numpy = (pred_d / pred_d.max()) * 255
    pred_d_numpy[0:cut.item(0),:] = 1000
    #print(pred_d_numpy.shape[0]-down_cut.item(0),pred_d_numpy.shape[0])
    pred_d_numpy[pred_d_numpy.shape[0]-down_cut.item(0):pred_d_numpy.shape[0],:] = 0
    # Put less lines to see the result
    # i = cut.item(0) + 2
    # max = pred_d_numpy.shape[0]-down_cut.item(0)-4
    # while i < max:
    #     pred_d_numpy[i,:] = 0
    #     pred_d_numpy[i+1,:] = 0
    #     pred_d_numpy[i+2,:] = 0
    #     i += 4
    return (pred_d_numpy).astype('uint16')
            
        