import argparse
from functools import partial
from os import path
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
from dataset.dataset import SegmentationDataset, segmentation_collate#, TrainingSegmentationDataset
from dataset.transform import SegmentationTransform
from modules.deeplab import DeeplabV3
#from modules.bn import InPlaceABN
from inplace_abn import InPlaceABN
import pdb
import cv2

parser = argparse.ArgumentParser(description="Testing script for the Vistas segmentation model")
parser.add_argument("--scales", metavar="LIST", type=str, default="[0.7, 1, 1.2]", help="List of scales")
parser.add_argument("--flip", action="store_true", help="Use horizontal flipping")
parser.add_argument("--fusion-mode", metavar="NAME", type=str, choices=["mean", "voting", "max"], default="mean",
                    help="How to fuse the outputs. Options: 'mean', 'voting', 'max'")
parser.add_argument("--output-mode", metavar="NAME", type=str, choices=["palette", "raw", "prob"],
                    default="final",
                    help="How the output files are formatted."
                         " -- palette: color coded predictions"
                         " -- raw: gray-scale predictions"
                         " -- prob: gray-scale predictions plus probabilities")
parser.add_argument("snapshot", metavar="SNAPSHOT_FILE", type=str, help="Snapshot file to load")
parser.add_argument("data", metavar="IN_DIR", type=str, help="Path to dataset")
parser.add_argument("output", metavar="OUT_DIR", type=str, help="Path to output folder")
parser.add_argument("--world-size", metavar="WS", type=int, default=1, help="Number of GPUs")
parser.add_argument("--rank", metavar="RANK", type=int, default=0, help="GPU id")

random.seed(42)

def save_preds(log_time, preds, img_timestamp):
    f= open("output_train/preds_output_" +log_time+ ".txt","a+")
#    preds_array = preds[0].data.cpu().numpy()
    preds_array = preds.squeeze().tolist()
    for i in range(len(preds_array)):
        f.write(str(preds_array[i]) +" " )
    f.write(str(img_timestamp)+"\n")
    del preds_array

def save_output(log_time, image_folder_string, target, preds, preds_eval, logstring):
    f= open("output_train/output_" +log_time+ ".txt","a+")
#    target_array = target.data.cpu().numpy()
    target_array = target.squeeze().tolist()

    f.write(str(image_folder_string)+"\n")
    f.write("Desejado: ")
    for i in range(len(target_array)):
        f.write(str(target_array[i]) + " " )
    del target_array
#    preds_array = preds[0].data.cpu().numpy()
    preds_array = preds.squeeze().tolist()

    f.write("\nPrevisto: ")
    for i in range(len(preds_array)):
        f.write(str(preds_array[i]) + " " )
    f.write("\n")
    del preds_array
    if(preds_eval!=0):
#        preds_eval_array = preds_eval[0].data.cpu().numpy()
        preds_eval_array = preds_eval.squeeze().tolist()
        f.write("Eval: ")
        for i in range(len(preds_eval_array)):
            f.write(str(preds_eval_array[i]) + " " )
        f.write("\n"+logstring+"\n")
        del preds_eval_array




def get_data(txt_rddf, image_folder, arg_random):
    dados_rddf = []
    if('/' == image_folder[-1]):
        image_folder = image_folder[:-1]
    with open(txt_rddf, 'r') as f:
        dados_rddf = [line.strip().split(" ") for line in f]
    images_path = []
    for i in range(len(dados_rddf)):
        images_path.append(image_folder + '/' + dados_rddf[i][5] + '-r.png')

#            print(image_folder + '/' + dados_rddf[i][5] + '-r.png')
    if(arg_random):
        lista_temp = []
        while(len(lista_temp) < 100):
            rand_number = random.randint(0,len(dados_rddf)-1)
#            print(len(dados_rddf), rand_number)
            if(dados_rddf[rand_number] not in lista_temp):
                lista_temp.append(dados_rddf[rand_number])
        dados_rddf = lista_temp
    return  dados_rddf, image_folder, images_path


def flip(x, dim):
    indices = [slice(None)] * x.dim()
    indices[dim] = torch.arange(x.size(dim) - 1, -1, -1,
                                dtype=torch.long, device=x.device)
    return x[tuple(indices)]

def weighted_mse_loss(input, target, weight):
    return torch.sum(weight * (input - target) ** 2)

class SegmentationModule(nn.Module):
    _IGNORE_INDEX = 255
    
    class _MeanFusion:
        def __init__(self, x, classes):
            self.buffer = x.new_zeros(x.size(0), classes, x.size(2), x.size(3))
            self.counter = 0

        def update(self, sem_logits):
            probs = functional.softmax(sem_logits, dim=1)
            self.counter += 1
            self.buffer.add_((probs - self.buffer) / self.counter)

        def output(self):
            probs, cls = self.buffer.max(1)
            return cls #zprobs, cls

    class _VotingFusion:
        def __init__(self, x, classes):
            self.votes = x.new_zeros(x.size(0), classes, x.size(2), x.size(3))
            self.probs = x.new_zeros(x.size(0), classes, x.size(2), x.size(3))

        def update(self, sem_logits):
            probs = functional.softmax(sem_logits, dim=1)
            probs, cls = probs.max(1, keepdim=True)

            self.votes.scatter_add_(1, cls, self.votes.new_ones(cls.size()))
            self.probs.scatter_add_(1, cls, probs)

        def output(self):
            cls, idx = self.votes.max(1, keepdim=True)
            probs = self.probs / self.votes.clamp(min=1)
            probs = probs.gather(1, idx)
            return probs.squeeze(1), cls.squeeze(1)

    class _MaxFusion:
        def __init__(self, x, _):
            self.buffer_cls = x.new_zeros(x.size(0), x.size(2), x.size(3), dtype=torch.long)
            self.buffer_prob = x.new_zeros(x.size(0), x.size(2), x.size(3))

        def update(self, sem_logits):
            probs = functional.softmax(sem_logits, dim=1)
            max_prob, max_cls = probs.max(1)

            replace_idx = max_prob > self.buffer_prob
            self.buffer_cls[replace_idx] = max_cls[replace_idx]
            self.buffer_prob[replace_idx] = max_prob[replace_idx]

        def output(self):
            return self.buffer_prob, self.buffer_cls

    def __init__(self, body, head, head_channels, classes, fusion_mode="mean"):
        super(SegmentationModule, self).__init__()
        self.body = body
        self.head = head
#        self.cls = nn.Conv2d(head_channels, classes, 1)
        # self.cls = nn.Conv2d(head_channels, classes, 3) #3x3 conv layer
        self.out_vector=nn.Linear(1228800, 5)
#        self.out_vector=nn.Sequential(
#            nn.ReLU(),
#            nn.Linear(1228800, 5)
#            nn.Linear(50 , 5)
#        )

        self.classes = classes
#        if fusion_mode == "mean":
#            self.fusion_cls = SegmentationModule._MeanFusion
#        elif fusion_mode == "voting":
#            self.fusion_cls = SegmentationModule._VotingFusion
#        elif fusion_mode == "max":
#            self.fusion_cls = SegmentationModule._MaxFusion

    def _network(self, x, scale):
        if scale != 1:
            scaled_size = [round(s * scale) for s in x.shape[-2:]]
            x_up = functional.upsample(x, size=scaled_size, mode="bilinear")
        else:
            x_up = x

        x_up = self.body(x_up)
        x_up = self.head(x_up)

        #pdb.set_trace()

        x_up = x_up.reshape(x_up.size(0), -1)
        sem_logits = self.out_vector(x_up)

        del x_up
        return sem_logits

    def forward(self, x, scales, do_flip=True):
        return self._network(x, 1)


def main():
    # Load configuration
    args = parser.parse_args()

    # Torch stuff
    #torch.cuda.set_device(args.rank)
    torch.cuda.set_device(0) # To get this to run on free RAAMAC GPU - Dominic
    cudnn.benchmark = True

    # Create model by loading a snapshot
    body, head, cls_state = load_snapshot('/home/sabrina/Documents/Inplace_ABN/wide_resnet38_deeplab_vistas.pth.tar')
    model = SegmentationModule(body, head, 256, 5, args.fusion_mode) # this changes
                                                                      # number of classes
                                                                      # in final model.cls layer
    arg_random = True

    my_dataset, image_folder, images_path = get_data('/dados/rddf_predict/listen_2019-11-29_11:32:36', '/dados/log_png_1003/', arg_random)
    my_dataset = np.array(my_dataset)
    data_imgs = my_dataset[:, -1]
#    print(data_imgs)
    data_target = my_dataset[:, :-1]
    data_target = data_target.astype(np.float)

#     Create data loader
    transformation = SegmentationTransform(     # Only applied to RGB
        640,
        (0.41738699, 0.45732192, 0.46886091), # rgb mean and std - would this affect training at all?
        (0.25685097, 0.26509955, 0.29067996),
    )

#    print(data_target)
    ####################
    #   TRAIN
    ####################
    
    # Run fine-tuning (of modified class layers)   
    for p in model.body.parameters():
        p.requires_grad = False
    for q in model.head.parameters():
        q.requires_grad = False
    for q in model.out_vector.parameters():
        q.requires_grad = True

#    for m in model.modules(): 
#        if isinstance(m, InPlaceABN):
#            m.affine = False


    #no_epochs = 50
    LR = 1e-7
#    momentum = 0.98
    epochs = 2000

    device = torch.device("cuda:0")
    model.to(device)
    model.train()



    # Am definitely training on the right parameters.

    #optimizer = optim.SGD(filter(lambda x: x.requires_grad, model.parameters()),
    #                    lr=LR,momentum = momentum) 

    optimizer = optim.Adam(filter(lambda x: x.requires_grad, model.parameters()),
                        lr=LR)   
    #n

    oname = 'Adam'
    
    scales = eval(args.scales)
#    lossfunction = weighted_mse_loss().to(device)
    #pdb.set_trace()
    
    log_time = str(time.time()) 
    logforloss = open('output_train/lossfunction_'+ log_time +'.txt','a')
    for epoch in range(epochs):

        #if epoch == 200:
        #    LR *= 0.1

        if epoch % 20 == 0:
            LR *= 0.1
 #       if epoch == 100:
 #          LR *= 0.1

        for batch_i, (d_img, d_target)  in enumerate(zip(data_imgs, data_target)):
            image_folder_string = (image_folder + '/' + str(d_img) + '-r.png')
            print(image_folder_string)
            image_temp = Image.open(image_folder_string).convert(mode="RGB")
            # normalize
            img = transformation(image_temp)
            target = torch.from_numpy(d_target).float().to(device)
            img  = img.unsqueeze(0).to(device)

            preds = model(img, scales, args.flip)
            preds_eval = preds
            loss = weighted_mse_loss(preds.float(),target.float(), torch.tensor([1.0, 5.0, 1.0, 1.0, 1.0]).to(device))
            print("Desejado: ",target , "\nPrevisto: " , preds)
            save_preds(log_time, preds, d_img)   
 #           with torch.no_grad():
 #               model.eval()
 #               preds_eval = model(img, scales, args.flip)
 #               print("Eval: " ,preds_eval, "\n")
 #               model.train()

            optimizer.zero_grad()
            loss.backward()

            optimizer.step()
            #torch.save(model.state_dict,'ckpoint_{}_{}.pt'.format(batch_i,epoch))
            logstring =  'Train Epoch: {} [/{} ({:.0f}%)]\tLoss: {:.6f}'.format(
                    epoch , len(my_dataset),
                    100. * batch_i / len(my_dataset), loss.item())
            print(logstring)

 #           save_output(log_time, image_folder_string, target, preds, preds_eval, logstring)
            save_output(log_time, image_folder_string, target, preds, 0, logstring)
            del preds, target, img, preds_eval
            logforloss.write(logstring + '\n') 
 #       torch.save(model.state_dict(),'weights/ckpoint_{}_{}_{}.pt'.format(0, LR, log_time))
        # Overwrite checkpoint
        torch.save(model.state_dict(),'weights/ckpoint_{}.pt'.format(log_time))


 #   torch.save(model.state_dict(),'ckpoint_{}_{}_{}.pt'.format(epoch, LR, time.time()))
    logforloss.close()
    
    
    #################
    #   TEST
    #################
    
    # Load checkpoint
    # data = torch.load('ckpoint_399_AdamwithLRdecay_1e-05.pt')
    
    # tmpdict = {'weight' : data['cls.weight'], 
    #            'bias': data['cls.bias']}
    # model.cls.load_state_dict(tmpdict)
    #pdb.set_trace()
    #model.cls.load_state_dict(data["state_dict"]["cls"])
    #modle.cls.weight = data([''])
#    model.eval()
    
#    with torch.no_grad():
#
#        for epoch in range(epochs):
#
#            #if epoch == 200:
#            #    LR *= 0.1
#
#            if epoch == 100:
#                LR *= 0.1
#
#            for batch_i, (d_img, d_target)  in enumerate(zip(data_imgs, data_target)):
#                print(image_folder + '/' + str(d_img) + '-r.png')
#                image_temp = Image.open(image_folder + '/' + d_img + '-r.png').convert(mode="RGB")
#            # normalize
##            image_temp = image_temp/255.0
#                img = transformation(image_temp)
##            image_temp = np.transpose(np.expand_dims(image_temp, axis=0), (0, 3, 1, 2))
##            img, target = torch.from_numpy(image_temp).float().to(device), torch.from_numpy(d_target).float().to(device)
#            # img = rec["img"].to(device)
#                target = torch.from_numpy(d_target).float().to(device)
#                optimizer.zero_grad()
#            #pdb.set_trace()
#                img  = img.unsqueeze(0).to(device)
#                
#                preds = model(img, scales, args.flip)
#                print(preds)
#
#                loss = lossfunction(preds.float(),target.float())
#
#                #print(d_img)
#                #print(preds.float(), '----', target.float())
#                #print(target.shape)
#                #print(preds.shape)
#
#                #torch.save(model.state_dict,'ckpoint_{}_{}.pt'.format(batch_i,epoch))
#                del preds, target, img
#    


def load_snapshot(snapshot_file):
    """Load a training snapshot"""
    print("--- Loading model from snapshot")

    # Create network
#    norm_act = partial(InPlaceABN, activation="leaky_relu", slope=.01)
    norm_act = partial(InPlaceABN, activation="leaky_relu", activation_param=.01)
    body = models.__dict__["net_wider_resnet38_a2"](norm_act=norm_act, dilation=(1, 2, 4, 4))
    head = DeeplabV3(4096, 256, 256, norm_act=norm_act, pooling_size=(84, 84))

    # Load snapshot and recover network state
    data = torch.load(snapshot_file)
    body.load_state_dict(data["state_dict"]["body"])
    head.load_state_dict(data["state_dict"]["head"])

    return body, head, data["state_dict"]["cls"]


label_names = ["Bird", "Ground Animal", "Curb", "Fence", "Guard Rail", "Barrier", "Wall", "Bike Lane",
               "Crosswalk - Plain", "Curb Cut", "Parking", "Pedestrian Area", "Rail Track", "Road",
               "Service Lane", "Sidewalk", "Bridge", "Building", "Tunnel", "Person", "Bicyclist",
               "Motorcyclist", "Other Rider", "Lane Marking - Crosswalk", "Lane Marking - General",
               "Mountain", "Sand", "Sky", "Snow", "Terrain", "Vegetation", "Water", "Banner", "Bench", 
               "Bike Rack", "Billboard", "Catch Basin", "CCTV Camera", "Fire Hydrant", "Junction Box",
               "Mailbox", "Manhole", "Phone Booth", "Pothole", "Street Light", "Pole", "Traffic Sign Frame", 
               "Utility Pole", "Traffic Light", "Traffic Sign (Back)", "Traffic Sign (Front)", "Trash Can", 
               "Bicycle", "Boat", "Bus", "Car", "Caravan", "Motorcycle", "On Rails", "Other Vehicle",
               "Trailer", "Truck", "Wheeled Slow", "Car Mount", "Ego Vehicle"]


_PALETTE = np.array([[165, 42, 42],
                     [0, 192, 0],
                     [196, 196, 196],
                     [190, 153, 153],
                     [180, 165, 180],
                     [90, 120, 150],
                     [102, 102, 156],
                     [128, 64, 255],
                     [140, 140, 200],
                     [170, 170, 170],
                     [250, 170, 160],
                     [96, 96, 96],
                     [230, 150, 140],
                     [128, 64, 128],
                     [110, 110, 110],
                     [244, 35, 232],
                     [150, 100, 100],
                     [70, 70, 70],
                     [150, 120, 90],
                     [220, 20, 60],
                     [255, 0, 0],
                     [255, 0, 100],
                     [255, 0, 200],
                     [200, 128, 128],
                     [255, 255, 255],
                     [64, 170, 64],
                     [230, 160, 50],
                     [70, 130, 180],
                     [190, 255, 255],
                     [152, 251, 152],
                     [107, 142, 35],
                     [0, 170, 30],
                     [255, 255, 128],
                     [250, 0, 30],
                     [100, 140, 180],
                     [220, 220, 220],
                     [220, 128, 128],
                     [222, 40, 40],
                     [100, 170, 30],
                     [40, 40, 40],
                     [33, 33, 33],
                     [100, 128, 160],
                     [142, 0, 0],
                     [70, 100, 150],
                     [210, 170, 100],
                     [153, 153, 153],
                     [128, 128, 128],
                     [0, 0, 80],
                     [250, 170, 30],
                     [192, 192, 192],
                     [220, 220, 0],
                     [140, 140, 20],
                     [119, 11, 32],
                     [150, 0, 255],
                     [0, 60, 100],
                     [0, 0, 142],
                     [0, 0, 90],
                     [0, 0, 230],
                     [0, 80, 100],
                     [128, 64, 64],
                     [0, 0, 110],
                     [0, 0, 70],
                     [0, 0, 192],
                     [32, 32, 32],
                     [120, 10, 10]], dtype=np.uint8)


# img = np.zeros((700,300,3), np.uint8)
# font = cv2.FONT_HERSHEY_SIMPLEX
# for i, label in enumerate(label_names):
#     ystride = 10    
#     #pdb.set_trace()
#     cv2.putText(img,label,(0,i*ystride + 30), font, 0.5,np.ndarray.tolist(_PALETTE[i]),1,cv2.LINE_AA)

# cv2.imwrite("labelcolors.png",img)


# _PALETTE = np.concatenate([_PALETTE, np.zeros((256 - _PALETTE.shape[0], 3), dtype=np.uint8)], axis=0)
# _PALETTE = ImagePalette.ImagePalette(
#     palette=list(_PALETTE[:, 0]) + list(_PALETTE[:, 1]) + list(_PALETTE[:, 2]), mode="RGB")





def get_pred_image(tensor, out_size, with_palette):
    tensor = tensor.numpy()
    if with_palette:
        img = Image.fromarray(tensor.astype(np.uint8), mode="P")
        img.putpalette(_PALETTE)
        
    else:
        img = Image.fromarray(tensor.astype(np.uint8), mode="L")

    return img.resize(out_size, Image.NEAREST)


def get_prob_image(tensor, out_size):
    tensor = (tensor * 255).to(torch.uint8)
    img = Image.fromarray(tensor.numpy(), mode="L")
    return img.resize(out_size, Image.NEAREST)


if __name__ == "__main__":
    main()
