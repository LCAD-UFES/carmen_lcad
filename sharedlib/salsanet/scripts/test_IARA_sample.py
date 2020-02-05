from __future__ import print_function, division
import os.path
from config import *
from utils import *
from model import *
from segmenter import SegmenterNet
import numpy as np

import matplotlib.pyplot as plt

cfg = lidar_config()
os.environ['CUDA_VISIBLE_DEVICES'] = str(cfg.GPU)
print("creating network model using gpu " + str(cfg.GPU))
def save_txt(data, file_name, tag):
  
  # Write the array to disk
  with open(os.path.join(os.getenv("CARMEN_HOME") + '/sharedlib/salsanet/data/', file_name+ tag +'.txt'), 'w') as outfile:
    # I'm writing a header here just for the sake of readability
    # Any line starting with "#" will be ignored by numpy.loadtxt
    outfile.write('# Array shape: {0}\n'.format(data.shape))

    # Iterating through a ndimensional array produces slices along
    # the last axis. This is equivalent to data[i,:,:] in this case
    for data_slice in data:

        # The formatting string indicates that I'm writing out
        # the values in left-justified columns 7 characters in width
        # with 2 decimal places.  
        np.savetxt(outfile, data_slice, fmt='%-7.2f', delimiter='\t')

        # Writing out a break to indicate different slices...
        outfile.write('# New slice\n')

def test_with_point_cloud(net, cfg, cloud_path):
    
    # load pc
    lidar = np.loadtxt(cloud_path, delimiter="\t")
    VERTICAL_RESOLUTION = 32
    NUM_POINTS = lidar.shape[0]
    
    print("lidar.shape={}".format(
        lidar.shape)) #(99928, 7)
    # minInColumns = np.amin(lidar, axis=0)
    # print('lidar: min value of every column: ', minInColumns)
    # maxInColumns = np.amax(lidar, axis=0)
    # print('lidar: max value of every column: ', maxInColumns)
    AZIMUTH_LEVEL = lidar.shape[0]//cfg.IMAGE_HEIGHT
    
    lidar = lidar.reshape((cfg.IMAGE_HEIGHT,AZIMUTH_LEVEL,5))
    #save_txt(lidar, "original", "_iara")
    lidar_mask = np.reshape(
            (lidar[:, :, 4] > 0),
            [cfg.IMAGE_HEIGHT, AZIMUTH_LEVEL, 1])
    print(lidar_mask.shape)
    lidar = np.append(lidar, lidar_mask, axis=2)

    lidar = lidar.reshape((NUM_POINTS,6))
    
    # define the region of interest for bird eye view image generation
    #Frontal View
    frontalView = PC2ImgConverter(imgChannel=cfg.IMAGE_CHANNEL, xRange=[0, 50], yRange=[-12, 12], zRange=[-10, 8],
                                                   xGridSize=0.2, yGridSize=0.3, zGridSize=0.3, maxImgHeight=cfg.IMAGE_HEIGHT,
                                                   maxImgWidth=cfg.IMAGE_WIDTH, maxImgDepth=64)
    #Rear View
    rearView = PC2ImgConverter(imgChannel=cfg.IMAGE_CHANNEL, xRange=[-50, 0], yRange=[-12, 12], zRange=[-10, 8],
                                                   xGridSize=0.2, yGridSize=0.3, zGridSize=0.3, maxImgHeight=cfg.IMAGE_HEIGHT,
                                                   maxImgWidth=cfg.IMAGE_WIDTH, maxImgDepth=64)
    

    bevFrontalImg, bevFrontalCloud = frontalView.getBEVImage(lidar)
    bevFrontalImg = bevFrontalImg.astype('float32') / 255
    bevRearImg, bevRearCloud = rearView.getBEVImage(lidar)
    bevRearImg = bevRearImg.astype('float32') / 255
    #print('bevCloud shape: ', bevCloud.shape) # (42419, 7)
    #save_txt(resultFrontalPointCloud, "test", "front")
    #resultFrontalPointCloud = resultFrontalPointCloud.reshape((AZIMUTH_LEVEL*cfg.IMAGE_HEIGHT))
    
    print('bird eye view image frontal shape: ', bevFrontalImg.shape) # (32, 256, 4)
    #showBevImg(bevFrontalImg)
    #showBevImg(bevRearImg)

    with tf.Session(graph=net.graph) as sess:
        net.initialize_vars(sess)
        
        pred_frontal_img = net.predict_single_image(input_img=bevFrontalImg, session=sess)
        pred_rear_img = net.predict_single_image(input_img=bevRearImg, session=sess)
        print('predicted image shape: ', pred_frontal_img.shape, ' type: ', pred_frontal_img.dtype, ' min val: ', pred_frontal_img.min(),
              ' max val: ', pred_frontal_img.max())
        pointCloudSegmented = np.zeros(NUM_POINTS)
        frontalView.getCloudsFromAnyImage(pred_frontal_img, lidar, pointCloudSegmented) 
        rearView.getCloudsFromAnyImage(pred_rear_img, lidar, pointCloudSegmented)

        pointCloudSegmented = pointCloudSegmented.reshape(VERTICAL_RESOLUTION,np.int(NUM_POINTS/VERTICAL_RESOLUTION))
        print('pointCloudSegmented.shape=', pointCloudSegmented.shape)
        # roadCloud, vehicleCloud = frontalView.getCloudsFromBEVImage(pred_frontal_img, bevFrontalCloud, postProcessing=True)
        # print('roadCloud.shape=', roadCloud.shape)
        # print('vehicleCloud.shape=', vehicleCloud.shape)

        # roadRCloud, vehicleRCloud = rearView.getCloudsFromBEVImage(pred_rear_img, bevRearCloud, postProcessing=True)
        # print('roadRearCloud.shape=', roadRCloud.shape)
        # print('vehicleRearCloud.shape=', vehicleRCloud.shape)

        showPredImg(pred_frontal_img, pred_rear_img)

def showPredImg(frontalimg=[], rearimg=[]):
    img = np.concatenate((rearimg, frontalimg),axis=1)
    showBevImg(img)

def showBevImg(img=[]):
    plt.imshow(img)
    plt.axis('off')

    plt.show()


def main():

    # get config params
    cfg = lidar_config()
    
    # get trained model checkpoints
    model_ckp_name = "../logs/salsaNet_trained/model.chk-300"

    # load the trained model
    net = SegmenterNet(cfg, model_ckp_name)

    # testing by using the raw point cloud data
    test_cloud = "../data/1542360706.267678.txt"
    test_with_point_cloud(net, cfg, test_cloud)

    return

if __name__ == '__main__':

    main()
