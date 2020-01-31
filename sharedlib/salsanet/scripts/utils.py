import random
import numpy as np
import math
import os.path
import skimage as sk
from glob import glob
from skimage.transform import rotate

def generate_lidar_batch_function(data_folder, channel_nbr, class_nbr, loss_weights, augmentation=False):

    """ Batch Generator Function to create batches from the training data """
    # number of images
    n_image = len(glob(os.path.join(data_folder, '*.npy')))

    def get_batches(batch_size):
        """ Create batches from the training data """
        images = glob(os.path.join(data_folder,  '*.npy'))
        random.shuffle(images)

        for batch_i in range(0, len(images), batch_size):
            image_batch = []
            gt_image_batch = []
            weight_batch = []
            for image_file in images[batch_i:batch_i+batch_size]:

                lidar_data = np.load(image_file)
                image = lidar_data[:,:,0:4] # mean, max, ref, den
                label_img = lidar_data[:,:,4] # ground truth

                # assign weight for each class
                weight_img = np.zeros(label_img.shape)
                for l in range(class_nbr):
                    weight_img[label_img == l] = loss_weights[int(l)]

                if augmentation:
                    if np.random.rand() > 0.5:
                        # flip image horizontally
                        flip_image = image[::-1, :, :]
                        flip_label = label_img[::-1, :]
                        flip_weight_img = weight_img[::-1, :]
                        image_batch.append(flip_image.astype('float32') / 255)
                        gt_image_batch.append(flip_label)
                        weight_batch.append(flip_weight_img)

                    elif np.random.rand() > 0.5:
                        # rotate image
                        random_degree = random.uniform(-5, 5)
                        rot_image = rotate(image, angle=random_degree, center=None, preserve_range=True, order=0, clip=True)
                        rot_label = rotate(label_img, angle=random_degree, center=None, preserve_range=True, order=0, clip=True)
                        rot_weight = rotate(weight_img, angle=random_degree, center=None, preserve_range=True, order=0, clip=True)

                        image_batch.append(rot_image.astype('float32') / 255)
                        gt_image_batch.append(rot_label) 
                        weight_batch.append(rot_weight)

                    else:
                        image_batch.append(image.astype('float32') / 255)
                        gt_image_batch.append(label_img)
                        weight_batch.append(weight_img)

                else:
                    image_batch.append(image.astype('float32') / 255)
                    gt_image_batch.append(label_img)
                    weight_batch.append(weight_img)

            yield np.array(image_batch), np.array(gt_image_batch), np.array(weight_batch)

    return get_batches, n_image

def sd_calc(data):
    n = len(data)

    if n <= 1:
        return 0.0

    mean, sd = avg_calc(data), 0.0

    # calculate stan. dev.
    for el in data:
        sd += (float(el) - mean)**2
    sd = math.sqrt(sd / float(n-1))

    return sd

def avg_calc(ls):
    n, mean = len(ls), 0.0

    if n <= 1:
        return ls[0]

    # calculate average
    for el in ls:
        mean = mean + float(el)
    mean = mean / float(n)

    return mean

def convertMean(input):
    output = input

    for i in range(0, input.shape[0]):
        for j in range(0, input.shape[1]):
            p = input[i, j]
            output[i,j] = MapHeightToGrayscale(p)

    return output

def MapHeightToGrayscale(currHeight):

    medianRoadHeight = -1.6
    minHeight = -3
    maxHeight = 3
    delta = (maxHeight - minHeight) / 256.0
    deltaHeight = currHeight - medianRoadHeight;
    grayLevel = 0

    if deltaHeight >= maxHeight:
        grayLevel = 255
    elif deltaHeight <= minHeight:
        grayLevel = 0
    else:
        grayLevel = np.floor((deltaHeight - minHeight) / delta)

    if currHeight == 0:
        grayLevel = 0

    return grayLevel

def convertStd(input):
    output = input

    for i in range(0, input.shape[0]):
        for j in range(0, input.shape[1]):
            p = input[i, j]
            output[i,j] = MapStdToGrayscale(p)

    return output

def MapStdToGrayscale(std):

    minStd = 0
    maxStd = 1
    delta = (maxStd - minStd) / 256.0
    grayLevel = 0

    if std >= maxStd:
        grayLevel = 255
    elif std <= minStd:
        grayLevel = 0
    else:
        grayLevel = np.floor((std - minStd) / delta)

    return grayLevel

def convertDensity(input):
    output = input

    for i in range(0, input.shape[0]):
        for j in range(0, input.shape[1]):
            p = input[i, j]
            output[i,j] = MapDensityToGrayscale(p)

    return output

def MapDensityToGrayscale(density):

    minDensity = 0
    maxDensity = 16
    delta = (maxDensity - minDensity) / 256.0
    grayLevel = 0

    if density >= maxDensity:
        grayLevel = 255
    elif density <= minDensity:
        grayLevel = 0
    else:
        grayLevel = np.floor((density - minDensity) / delta)

    return grayLevel

def convertReflectivity(input):

    output = np.round(input*255)

    return output

class PC2ImgConverter(object):

    def __init__(self, imgChannel=4, xRange=[0, 100], yRange=[-10, 10], zRange=[-10, 10], xGridSize=0.1, yGridSize=0.1,
                 zGridSize=0.1, maxImgWidth=512, maxImgHeight=64, maxImgDepth=64):

        self.xRange = xRange
        self.yRange = yRange
        self.zRange = zRange
        self.xGridSize = xGridSize
        self.yGridSize = yGridSize
        self.zGridSize = zGridSize
        self.topViewImgWidth = np.int((xRange[1] - xRange[0]) / xGridSize)
        self.topViewImgHeight = np.int((yRange[1] - yRange[0]) / yGridSize)
        self.topViewImgDepth = np.int((zRange[1] - zRange[0]) / zGridSize)
        self.frontViewImgWidth = np.int((zRange[1] - zRange[0]) / zGridSize)
        self.frontViewImgHeight = np.int((yRange[1] - yRange[0]) / yGridSize)
        self.imgChannel = imgChannel
        self.maxImgWidth = maxImgWidth
        self.maxImgHeight = maxImgHeight
        self.maxImgDepth = maxImgDepth
        self.maxDim = 5000

        if self.topViewImgWidth > self.maxImgWidth or self.topViewImgHeight > self.maxImgHeight or self.topViewImgDepth > self.maxImgDepth:
            print("ERROR in top view image dimensions mismatch")

    def getBEVImage(self, pointCloud):

        """ top view x-y projection of the input point cloud"""
        """ max image size maxImgWidth=512 times maxImgHeight=64 """
        topViewImage = np.zeros(shape=(self.maxImgHeight, self.maxImgWidth, self.imgChannel))

        imgMean = np.zeros(shape=(self.maxImgHeight, self.maxImgWidth))
        imgMax = np.zeros(shape=(self.maxImgHeight, self.maxImgWidth))
        imgRef = np.zeros(shape=(self.maxImgHeight, self.maxImgWidth))
        imgDensity = np.zeros(shape=(self.maxImgHeight, self.maxImgWidth))

        tempMatrix = np.empty(shape=(self.maxImgHeight, self.maxImgWidth, self.maxDim), dtype=np.float32)
        tempMatrix[:] = np.nan
        refMatrix = np.empty(shape=(self.maxImgHeight, self.maxImgWidth, self.maxDim), dtype=np.float32)
        refMatrix[:] = np.nan
        topViewPoints = []

        # compute top view points
        for p in range(0, len(pointCloud)):

            xVal = pointCloud[p][0]
            yVal = pointCloud[p][1]
            zVal = pointCloud[p][2]
            iVal = pointCloud[p][3]  # must be between 0 and 1

            if self.xRange[0] < xVal < self.xRange[1] and self.yRange[0] < yVal < self.yRange[1] and self.zRange[0] < zVal < self.zRange[1]:
                topViewPoints.append(pointCloud[p])
                pixelX = np.int(np.floor((xVal - self.xRange[0]) / self.xGridSize))
                pixelY = np.int(self.topViewImgHeight - np.floor((yVal - self.yRange[0]) / self.yGridSize))
                imgDensity[pixelY,pixelX] += 1
                indexVal = np.int(imgDensity[pixelY,pixelX])
                if indexVal>= self.maxDim:
                    print ("ERROR in top view image computation: indexVal " + str(indexVal) + " is greater than maxDim " + str(self.maxDim))
                tempMatrix[pixelY, pixelX, indexVal] = zVal
                refMatrix[pixelY, pixelX, indexVal] = iVal

        # compute statistics
        for i in range(0, self.maxImgHeight):
            for j in range(0, self.maxImgWidth):
                currPixel = tempMatrix[i,j,:]
                currPixel = currPixel[~np.isnan(currPixel)]   # remove nans

                currRef = refMatrix[i,j,:]
                currRef = currRef[~np.isnan(currRef)]   # remove nans

                if len(currPixel):
                    imgMean[i,j] = np.mean(currPixel)
                    imgMax[i,j] = np.max(currPixel)
                    imgRef[i,j] = np.mean(currRef)

        # convert to gray scale
        grayMean = convertMean(imgMean)
        grayMax = convertMean(imgMax)
        grayRef = convertReflectivity(imgRef)
        grayDensity = convertDensity(imgDensity)

        # place all computed images in a specific order
        topViewImage[:, :, 0] = grayMean
        topViewImage[:, :, 1] = grayMax
        topViewImage[:, :, 2] = grayRef
        topViewImage[:, :, 3] = grayDensity
        topViewCloud = np.asarray(topViewPoints)

        return topViewImage, topViewCloud

    def getCloudsFromBEVImage(self, predImg, topViewCloud, postProcessing = False):
        """ crop topviewcloud based on the network prediction image  """
        roadPoints = []
        vehPoints = []

        for p in range(0, len(topViewCloud)):

            xVal = topViewCloud[p][0]
            yVal = topViewCloud[p][1]
            zVal = topViewCloud[p][2]
            pixelX = np.int(np.floor((xVal - self.xRange[0]) / self.xGridSize))
            pixelY = np.int(self.topViewImgHeight - np.floor((yVal - self.yRange[0]) / self.yGridSize))
            classVal = predImg[pixelY, pixelX]


            if classVal == 1:
                roadPoints.append(topViewCloud[p])
            elif classVal == 2:
                vehPoints.append(topViewCloud[p])

        roadCloud = np.asarray(roadPoints)
        vehCloud = np.asarray(vehPoints)

        if postProcessing:

            # first global thresholding, make all points above 3  m as background
            globalThreshold = 3
            if len(roadCloud):
                roadCloud = roadCloud[roadCloud[:, 2] < globalThreshold]
            if len(vehCloud):
                vehCloud = vehCloud[vehCloud[:, 2] < globalThreshold]

            # second, apply thresholding only to road points
            # e.g. compute the mean of road points and remove those that are above
            if len(roadCloud):
                meanRoadZ = roadCloud[:, 2].mean()  # mean of third column, i.e. z values
                stdRoadZ = roadCloud[:, 2].std()  # mean of third column, i.e. z values
                roadThreshold = meanRoadZ + (1.0 * stdRoadZ)

                #print ("meanRoadZ: " + str(meanRoadZ) + " stdRoadZ: " + str(stdRoadZ) + " roadThreshold: " + str(roadThreshold))
                roadCloud = roadCloud[roadCloud[:, 2] < roadThreshold]


        return roadCloud, vehCloud


