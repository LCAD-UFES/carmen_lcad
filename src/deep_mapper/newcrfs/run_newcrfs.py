from __future__ import absolute_import, division, print_function
import torch
import torch.nn as nn
from torch.autograd import Variable

import os, sys, errno
import argparse
import time
import numpy as np
import cv2
import matplotlib.pyplot as plt
from tqdm import tqdm

from utils import post_process_depth, flip_lr
from networks.NewCRFDepth import NewCRFDepth
from dataloaders.dataloader import NewDataLoader
from collections import OrderedDict
import torchvision.transforms as transforms

def activate_virtual_environment(environment_root):
    """Configures the virtual environment starting at ``environment_root``."""
    activate_script = os.path.join(
        environment_root, 'bin', 'activate_this.py')
    exec(compile(open(activate_script, "rb").read(), activate_script, 'exec'), dict(__file__=activate_script))

carmen_home = os.getenv("CARMEN_HOME")
virtualenv_root = carmen_home + "/src/deep_mapper/newcrfs/venv"
activate_virtual_environment(virtualenv_root)

global model
global device

def initialize():
    global model
    global device
    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("NeWCRFs: device: %s" % device)
    print("NeWCRFs: Define Model with kitti")
    
    model = NewCRFDepth(version='large07', inv_depth=False, max_depth=80.0).to(device)
    model = torch.nn.DataParallel(model)
    
    checkpoint = torch.load(carmen_home + '/src/deep_mapper/newcrfs/ckpt/model_kittieigen.ckpt')
    model.load_state_dict(checkpoint['model'])
    model.eval()

    #num_params = sum([np.prod(p.size()) for p in model.parameters()])
    #print("Total number of parameters: {}".format(num_params))
    
    print("\n\n-------------------------------------------------------")
    print("       Pretrained Model NeWCRFs loaded!")
    print("-------------------------------------------------------\n\n")


def newcrfs_process_image(image, cut, down_cut):
    global model
    global device
    
    # print("NeWCRFs: Inference & Evaluate")
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
    input_RGB = Variable(batch['image'].to(device))

    with torch.no_grad():
        depth_est = model(input_RGB)
        post_process = False
        if post_process:
            image_flipped = flip_lr(image)
            depth_est_flipped = model(image_flipped)
            depth_est = post_process_depth(depth_est, depth_est_flipped)
        pred_d_numpy = depth_est.squeeze().cpu().numpy() * 256.0
        
    #print(pred_d)
    pred_d_numpy = (pred_d_numpy / pred_d_numpy.max()) * 256.0 * 1.256
    #pred_d_numpy = (pred_d_numpy / pred_d_numpy.max()) * 400.0
    pred_d_numpy[0:cut.item(0),:] = 0
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
            
        