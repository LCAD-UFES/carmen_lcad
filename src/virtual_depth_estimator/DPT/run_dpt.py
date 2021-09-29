#Activating virtualenv
import os

def activate_virtual_environment(environment_root):
    """Configures the virtual environment starting at ``environment_root``."""
    activate_script = os.path.join(
        environment_root, 'bin', 'activate_this.py')
    exec(compile(open(activate_script, "rb").read(), activate_script, 'exec'), dict(__file__=activate_script))

carmen_home = os.getenv("CARMEN_HOME")
virtualenv_root = carmen_home + "/src/virtual_depth_estimator/DPT/venv"
dpt_weight_path = carmen_home + "/src/virtual_depth_estimator/DPT/weights/"
output_path = carmen_home + "/src/virtual_depth_estimator/DPT/output/"
activate_virtual_environment(virtualenv_root)
#virtualenv activated
"""Compute depth maps for images in the input folder.
"""
import os
import glob
import torch
import cv2
import argparse


import util.io

from torchvision.transforms import Compose

from dpt.models import DPTDepthModel
from dpt.midas_net import MidasNet_large
from dpt.transforms import Resize, NormalizeImage, PrepareForNet
from PIL import Image

global model
global device
global transform

def initialize():
    global model
    global device
    global transform
    """Run MonoDepthNN to compute depth maps.

    Args:
        input_path (str): path to input folder
        output_path (str): path to output folder
        model_path (str): path to saved model
    """
    print("initialize")

    # select device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("device: %s" % device)

    # load network
    model_path = dpt_weight_path + "dpt_hybrid-midas-501f0c75.pt"
    net_w = net_h = 384
    model = DPTDepthModel(
        path=model_path,
        backbone="vitb_rn50_384",
        non_negative=True,
        enable_attention_hooks=False,
    )
    normalization = NormalizeImage(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
    transform = Compose(
        [
            Resize(
                net_w,
                net_h,
                resize_target=None,
                keep_aspect_ratio=True,
                ensure_multiple_of=32,
                resize_method="minimal",
                image_interpolation_method=cv2.INTER_CUBIC,
            ),
            normalization,
            PrepareForNet(),
        ]
    )

    model.eval()

    if device == torch.device("cuda"):
        model = model.to(memory_format=torch.channels_last)
        model = model.half()

    model.to(device)

    # get input
    # img_names = glob.glob(os.path.join(input_path, "*"))
    # num_images = len(img_names)

    # create output folder
    # os.makedirs(output_path, exist_ok=True)

    print("\n\n-------------------------------------------------------")
    print("       Pretrained Model DPT loaded!")
    print("-------------------------------------------------------\n\n")


def dpt_process_image(carmen_image, timestamp):
    global model
    global device
    global transform
    # converter a imagem
    # print ("opaaaa!! entrou no dpt_process_image")
    img = carmen_image
    # Convert RGB to BGR 
    img = img[:, :, ::-1].copy() 
    # print(img.shape)
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) / 255.0
    img_input = transform({"image": img})["image"]
    # compute
    with torch.no_grad():
        sample = torch.from_numpy(img_input).to(device).unsqueeze(0)
        if device == torch.device("cuda"):
            sample = sample.to(memory_format=torch.channels_last)
            sample = sample.half()
        prediction = model.forward(sample)
        prediction = (
            torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=img.shape[:2],
                mode="bicubic",
                align_corners=False,
            )
            .squeeze()
            .cpu()
            .numpy()
        )
    
    filename = output_path + str(timestamp.item(0))
    absolute_depth=False
    out = util.io.depth_to_img(filename, prediction, bits=2, absolute_depth=absolute_depth)
    # Window name in which image is displayed
    # window_name = 'DPT'
    # cv2.imshow(window_name, out)
    # cv2.waitKey(0) 
    return out

# if __name__ == "__main__":
#     main()
