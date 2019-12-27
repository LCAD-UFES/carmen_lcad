import argparse
from functools import partial
from os import path
import os
import time
import numpy as np
import torch
import torch.backends.cudnn as cudnn
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as functional
from PIL import Image, ImagePalette
from torch.utils.data import DataLoader, TensorDataset
from torch.utils.data.distributed import DistributedSampler
from torch.autograd import Variable
import math
import random
import models
# , TrainingSegmentationDataset
from dataset.dataset import SegmentationDataset, segmentation_collate, RDDFPredictDataset
from dataset.transform import SegmentationTransform
from modules.deeplab import DeeplabV3
#from modules.bn import InPlaceABN
from inplace_abn import InPlaceABN
import pdb
import cv2
# from ttictoc import TicToc

# t = TicToc()
# t.tic()

global model
global transformation
global device

def initialize(horizontal_resolution):
    print "ok, entrou no run_inplace_abn"

def initialize_ok(horizontal_resolution):
    global model
    global transformation
    global device
    chk_path = "output_batch_train/1576703507.3473513/checkpoints/ckpoint_1576703507.3473513_2.pt"
    # Torch stuff
    # torch.cuda.set_device(args.rank)
    torch.cuda.set_device(0)  # To get this to run on free RAAMAC GPU - Dominic
    cudnn.benchmark = True

    # Create model by loading a snapshot
    body, head, cls_state = load_snapshot(
        '/home/sabrina/Documents/Inplace_ABN/wide_resnet38_deeplab_vistas.pth.tar')
    model = SegmentationModule(body, head)  # this changes
    # number of classes
    # in final model.cls layer
    arg_random = True

#     Create data loader
    transformation = SegmentationTransform(     # Only applied to RGB
        horizontal_resolution,
        # rgb mean and std - would this affect training at all?
        (0.41738699, 0.45732192, 0.46886091),
        (0.25685097, 0.26509955, 0.29067996),
    )

    #image_folder, images_path = get_data('/dados/log_png_1003/')

    ####################
    #   INFERENCE
    ####################

    device = torch.device("cuda:0")
    model.to(device)

    model.eval()
    if(chk_path != ""):
        data = torch.load(chk_path)
        model.load_state_dict(data)
        model.to(device)
    print ("\n\n-------------------------------------------------------")
    print ("       Pretrained Model Inplace_abn loaded!")
    print ("-------------------------------------------------------\n\n")

def inplace_abn_process_image(d_img):
    #converter a imagem
    image_temp = Image.open(d_img).convert(mode="RGB")
    img = transformation(image_temp)
    img = img.unsqueeze(0).to(device, non_blocking=True)
    preds = model(img)
    print(d_img)
    print(preds)
    # t.toc()
    # print(t.elapsed)
    # t.tic()
    print("preds.shape={}".format(
        preds.shape))
    print("preds.type={}".format(preds.type))
    return preds

#parser = argparse.ArgumentParser(description="Testing script for the Vistas segmentation model")
#parser.add_argument("--scales", metavar="LIST", type=str, default="[0.7, 1, 1.2]", help="List of scales")
#parser.add_argument("--flip", action="store_true", help="Use horizontal flipping")
# parser.add_argument("--fusion-mode", metavar="NAME", type=str, choices=["mean", "voting", "max"], default="mean",
#                    help="How to fuse the outputs. Options: 'mean', 'voting', 'max'")
# parser.add_argument("--output-mode", metavar="NAME", type=str, choices=["palette", "raw", "prob"],
#                    default="final",
#                    help="How the output files are formatted."
#                         " -- palette: color coded predictions"
#                         " -- raw: gray-scale predictions"
#                         " -- prob: gray-scale predictions plus probabilities")
#parser.add_argument("snapshot", metavar="SNAPSHOT_FILE", type=str, help="Snapshot file to load")
#parser.add_argument("data", metavar="IN_DIR", type=str, help="Path to dataset")
#parser.add_argument("output", metavar="OUT_DIR", type=str, help="Path to output folder")
#parser.add_argument("--world-size", metavar="WS", type=int, default=1, help="Number of GPUs")
#parser.add_argument("--rank", metavar="RANK", type=int, default=0, help="GPU id")


def get_data(image_folder):
    if('/' == image_folder[-1]):
        image_folder = image_folder[:-1]
    images_path = []
    lista = os.listdir(image_folder)
    for i in range(len(lista)):
        images_path.append(image_folder + '/' + lista[i])
    return image_folder, images_path

# def flip(x, dim):
#    indices = [slice(None)] * x.dim()
#    indices[dim] = torch.arange(x.size(dim) - 1, -1, -1,
#                                dtype=torch.long, device=x.device)
#    return x[tuple(indices)]

# def weighted_mse_loss(input, target, weight):
#    return torch.sum(weight * (input - target) ** 2)


class SegmentationModule(nn.Module):
    _IGNORE_INDEX = 255

    def __init__(self, body, head):
        super(SegmentationModule, self).__init__()
        self.body = body
        self.head = head
        self.out_vector = nn.Linear(1228800, 4)
#        self.out_vector=nn.Sequential(
#            nn.ReLU(),
#            nn.Linear(1228800, 5)
#            nn.Linear(50 , 5)
#        )

    def _network(self, x):
        x_up = x

        x_up = self.body(x_up)
        x_up = self.head(x_up)
        x_up = x_up.reshape(x_up.size(0), -1)

        sem_logits = self.out_vector(x_up)

        del x_up
        return sem_logits

    def forward(self, x):
        return self._network(x)



def main():
    # Load configuration
    #    args = parser.parse_args()
    chk_path = "output_batch_train/1576703507.3473513/checkpoints/ckpoint_1576703507.3473513_2.pt"

    # Torch stuff
    # torch.cuda.set_device(args.rank)
    torch.cuda.set_device(0)  # To get this to run on free RAAMAC GPU - Dominic
    cudnn.benchmark = True

    # Create model by loading a snapshot
    body, head, cls_state = load_snapshot(
        '/home/sabrina/Documents/Inplace_ABN/wide_resnet38_deeplab_vistas.pth.tar')
    model = SegmentationModule(body, head)  # this changes
    # number of classes
    # in final model.cls layer
    arg_random = True

#     Create data loader
    transformation = SegmentationTransform(     # Only applied to RGB
        640,
        # rgb mean and std - would this affect training at all?
        (0.41738699, 0.45732192, 0.46886091),
        (0.25685097, 0.26509955, 0.29067996),
    )

    image_folder, images_path = get_data('/dados/log_png_1003/')

    ####################
    #   TRAIN
    ####################

    device = torch.device("cuda:0")
    model.to(device)

    model.eval()
    if(chk_path != ""):
        data = torch.load(chk_path)
        model.load_state_dict(data)
        model.to(device)
    with torch.no_grad():
        for batch_i, d_img in enumerate(images_path):
            image_temp = Image.open(d_img).convert(mode="RGB")

            img = transformation(image_temp)
            img = img.unsqueeze(0).to(device, non_blocking=True)

            preds = model(img)
            print(d_img)
            print(preds)
            t.toc()
            print(t.elapsed)
            t.tic()





def load_snapshot(snapshot_file):
    """Load a training snapshot"""
    print("--- Loading model from snapshot")

    # Create network
#    norm_act = partial(InPlaceABN, activation="leaky_relu", slope=.01)
    norm_act = partial(InPlaceABN, activation="leaky_relu",
                       activation_param=.01)
    body = models.__dict__["net_wider_resnet38_a2"](
        norm_act=norm_act, dilation=(1, 2, 4, 4))
    head = DeeplabV3(4096, 256, 256, norm_act=norm_act, pooling_size=(84, 84))

    # Load snapshot and recover network state
    data = torch.load(snapshot_file)
    body.load_state_dict(data["state_dict"]["body"])
    head.load_state_dict(data["state_dict"]["head"])

    return body, head, data["state_dict"]["cls"]


if __name__ == "__main__":
    main()
