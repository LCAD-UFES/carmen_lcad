from __future__ import print_function, division
import os.path
from config import *
from utils import *
from model import *
from segmenter import SegmenterNet

import matplotlib.pyplot as plt

cfg = lidar_config()
cfg.IMAGE_HEIGHT = 64
os.environ['CUDA_VISIBLE_DEVICES'] = str(cfg.GPU)
print("creating network model using gpu " + str(cfg.GPU))

def test_with_bev_images (net, img):
    cfg.IMAGE_HEIGHT = 64
    with tf.Session(graph=net.graph) as sess:
        net.initialize_vars(sess)

        print("loading bev img " + str(img))
        lidar_data = np.load(img)
        
        print("lidar_data.shape={}".format(
            lidar_data.shape))
        netInput = lidar_data[:,:,0:4] # mean, max, ref, den
        netInput = netInput.astype('float32') / 255

        pred_img = net.predict_single_image(input_img=netInput, session=sess)
        print('predicted image shape: ', pred_img.shape, ' type: ',  pred_img.dtype, ' min val: ',  pred_img.min(), ' max val: ', pred_img.max())

        showPredImg(pred_img)

def test_with_point_cloud(net, cfg, cloud_path):
    cfg.IMAGE_HEIGHT = 64
    # load pc
    pointCloud = np.load(cloud_path)
    # print("pointcloud.shape={}".format(
    #     pointCloud.shape)) #(99928, 7)
    # minInColumns = np.amin(pointCloud, axis=0)
    # print('pointCloud: min value of every column: ', minInColumns)
    # maxInColumns = np.amax(pointCloud, axis=0)
    # print('rear: max value of every column: ', maxInColumns)
    # define the region of interest for bird eye view image generation
    pc2img = PC2ImgConverter(imgChannel=cfg.IMAGE_CHANNEL, xRange=[0, 50], yRange=[-6, 12], zRange=[-10, 8],
                                                   xGridSize=0.2, yGridSize=0.3, zGridSize=0.3, maxImgHeight=cfg.IMAGE_HEIGHT,
                                                   maxImgWidth=cfg.IMAGE_WIDTH, maxImgDepth=64)
    
    bevImg, bevCloud = pc2img.getBEVImage(pointCloud)
    bevImg = bevImg.astype('float32') / 255
    print('bevCloud shape: ', bevCloud.shape) # (42419, 7)
    print('bird eye view image shape: ', bevImg.shape) # (64, 512, 4)
    #save_txt(bevImg, "pc", "_out")

    with tf.Session(graph=net.graph) as sess:
        net.initialize_vars(sess)

        pred_img = net.predict_single_image(input_img=bevImg, session=sess)
        print('predicted image shape: ', pred_img.shape, ' type: ', pred_img.dtype, ' min val: ', pred_img.min(),
              ' max val: ', pred_img.max())


        roadCloud, vehicleCloud = pc2img.getCloudsFromBEVImage(pred_img, bevCloud, postProcessing=True)

        showPredImg(pred_img)

def showPredImg(img=[]):

    plt.imshow(img)
    plt.axis('off')

    plt.show()

def main():

    # get config params
    cfg = lidar_config()
    cfg.IMAGE_HEIGHT = 64
    # get trained model checkpoints
    model_ckp_name = "../logs/salsaNet_trained/model.chk-300"

    # load the trained model
    net = SegmenterNet(cfg, model_ckp_name)

    # testing with the already created bird eye view images
    # test_bev_img = "../data/test_bev_image.npy"
    # test_with_bev_images(net, test_bev_img)

    # testing by using the raw point cloud data
    test_cloud = "../data/test_cloud.npy"
    test_with_point_cloud(net, cfg, test_cloud)

    return

if __name__ == '__main__':

    main()
