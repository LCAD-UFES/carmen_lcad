from email.policy import strict
import torch
import torchvision
from collections import OrderedDict
from models.model import GLPDepth
import torchvision.transforms as transforms
import cv2
import os

# An instance of your model.
carmen_home = os.getenv("CARMEN_HOME")

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
#device = torch.device("cpu")
print("GLPDepth: device: %s" % device)
print("GLPDepth: Define Model with kitti")
model = GLPDepth(max_depth=80.0, is_train=False).to(device)
model_weight = torch.load(carmen_home + '/src/deep_mapper/GLPDepth/ckpt/best_model_kitti.ckpt')
if 'module' in next(iter(model_weight.items()))[0]:
    model_weight = OrderedDict((k[7:], v) for k, v in model_weight.items())
model.load_state_dict(model_weight)
model.eval()

example = torch.rand(1, 3, 128, 128)
example = example.to(device)

traced_script_module = torch.jit.trace(model, example, strict=False)
traced_script_module.save(carmen_home + "/src/deep_mapper/GLPDepth/ckpt/best_model_kitti_cuda.pt")


