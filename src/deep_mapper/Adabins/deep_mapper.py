import os
import cv2
import numpy as np
from numpy.core.fromnumeric import resize
import torch
import torch.nn as nn
from PIL import Image
from torchvision import transforms
import model_io
from models import UnetAdaptiveBins


def activate_virtual_environment(environment_root):
    """Configures the virtual environment starting at ``environment_root``."""
    activate_script = os.path.join(
        environment_root, 'bin', 'activate_this.py')
    exec(compile(open(activate_script, "rb").read(), activate_script, 'exec'), dict(__file__=activate_script))

carmen_home = os.getenv("CARMEN_HOME")
virtualenv_root = carmen_home + "/src/deep_mapper/venv"
activate_virtual_environment(virtualenv_root)
#virtualenv activated


global inferHelper

def initialize():
        print('deep_mapper.py: inicializacao com nyu\n')
        global inferHelper 
        inferHelper = InferenceHelper('kitti')

def inferenceDepth(image):
        global inferHelper
        pred = inferHelper.predict_pil(image)
        print('deep_mapper.py: executou a predicao\n')
        return pred

def get_img_arr(image):
    im = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    im = cv2.resize(im, (640, 480))
    x = np.clip(np.asarray(im, dtype=float) / 255, 0, 1)
    return x

def _is_pil_image(img):
    return isinstance(img, Image.Image)

def _is_numpy_image(img):
    return isinstance(img, np.ndarray) and (img.ndim in {2, 3})

class ToTensor(object):
    def __init__(self):
        self.normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])

    def __call__(self, image, target_size=(640, 480)):
        # image = image.resize(target_size)
        image = self.to_tensor(image)
        image = self.normalize(image)
        return image

    def to_tensor(self, pic):
        if not (_is_pil_image(pic) or _is_numpy_image(pic)):
            raise TypeError(
                'pic should be PIL Image or ndarray. Got {}'.format(type(pic)))

        if isinstance(pic, np.ndarray):
            img = torch.from_numpy(pic.transpose((2, 0, 1)))
            return img

        # handle PIL Image
        if pic.mode == 'I':
            img = torch.from_numpy(np.array(pic, np.int32, copy=False))
        elif pic.mode == 'I;16':
            img = torch.from_numpy(np.array(pic, np.int16, copy=False))
        else:
            img = torch.ByteTensor(torch.ByteStorage.from_buffer(pic.tobytes()))
        # PIL image mode: 1, L, P, I, F, RGB, YCbCr, RGBA, CMYK
        if pic.mode == 'YCbCr':
            nchannel = 3
        elif pic.mode == 'I;16':
            nchannel = 1
        else:
            nchannel = len(pic.mode)
        img = img.view(pic.size[1], pic.size[0], nchannel)

        img = img.transpose(0, 1).transpose(0, 2).contiguous()
        if isinstance(img, torch.ByteTensor):
            return img.float()
        else:
            return img

class InferenceHelper:
    def __init__(self, dataset='nyu', device='cuda:0'):
        self.toTensor = ToTensor()
        self.device = device
        if dataset == 'nyu':
            self.min_depth = 1e-3
            self.max_depth = 10
            self.saving_factor = 1000  # used to save in 16 bit
            model = UnetAdaptiveBins.build(n_bins=256, min_val=self.min_depth, max_val=self.max_depth)
            pretrained_path = carmen_home+"/src/deep_mapper/Adabins/pretrained/AdaBins_nyu.pt"
        elif dataset == 'kitti':
            self.min_depth = 1e-3
            self.max_depth = 80
            self.saving_factor = 256
            model = UnetAdaptiveBins.build(n_bins=256, min_val=self.min_depth, max_val=self.max_depth)
            pretrained_path = carmen_home+"/src/deep_mapper/Adabins/pretrained/AdaBins_kitti.pt"
        else:
            raise ValueError("dataset can be either 'nyu' or 'kitti' but got {}".format(dataset))

        model, _, _ = model_io.load_checkpoint(pretrained_path, model)
        model.eval()
        self.model = model.to(self.device)
    
    @torch.no_grad()
    def predict_pil(self, image):
        # print("deep_mapper.py: predict_pil")
        
        height, width, layers = image.shape
        resized_img = ''
        cropped_img = ''
        if height < width :
            nw = (width * 640) / height
            dim = (int(nw),480)
            resized_img = cv2.resize(image,dim)
            w1 = int(nw/2 - 320)
            w2 = int(nw/2 + 320)
            cropped_img = resized_img[:,w1:w2]
        else:
            nh = (height * 640) / width
            dim = (640,int(nh))
            resized_img = cv2.resize(image,dim)        
            h1 = int(nh/2 - 240)
            h2 = int(nh/2 + 240)
            cropped_img = resized_img[h1:h2,:]

        #cv2.imwrite("/tmp/input2.png",pil_image)
        img = np.asarray(cropped_img) / 255.
        img = self.toTensor(img).unsqueeze(0).float().to(self.device)
        bin_centers, pred = self.predict(img)

        pred = pred.squeeze()
        out_min = np.min(pred)
        out_max = np.max(pred)
        pred = pred - out_min
        pred = pred/out_max
        
        #plasma = plt.get_cmap('plasma')
        #pred = plasma(pred)[:, :, :3]

        pred = (pred * 256).astype('uint16')
        #final = pred*255
        #cv2.imwrite("/tmp/output2.png",final)
        #print(final.shape)
        print(pred.shape)
        return (pred * 256).astype('uint16')
        #return bytearray(final)

    @torch.no_grad()
    def predict(self, image):
        bins, pred = self.model(image)
        pred = np.clip(pred.cpu().numpy(), self.min_depth, self.max_depth)

        # Flip
        image = torch.Tensor(np.array(image.cpu().numpy())[..., ::-1].copy()).to(self.device)
        pred_lr = self.model(image)[-1]
        pred_lr = np.clip(pred_lr.cpu().numpy()[..., ::-1], self.min_depth, self.max_depth)

        # Take average of original and mirror
        final = 0.5 * (pred + pred_lr)
        final = nn.functional.interpolate(torch.Tensor(final), image.shape[-2:],
                                          mode='bilinear', align_corners=True).cpu().numpy()

        final[final < self.min_depth] = self.min_depth
        final[final > self.max_depth] = self.max_depth
        final[np.isinf(final)] = self.max_depth
        final[np.isnan(final)] = self.min_depth

        centers = 0.5 * (bins[:, 1:] + bins[:, :-1])
        centers = centers.cpu().squeeze().numpy()
        centers = centers[centers > self.min_depth]
        centers = centers[centers < self.max_depth]

        return centers, final



    