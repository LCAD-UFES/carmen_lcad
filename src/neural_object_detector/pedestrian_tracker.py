#from __future__ import division
import os
#import re
import cv2
import sys
import numpy as np
#import matplotlib.pyplot as plt

global image_width
global image_height

def set_image_settings(w, h):
	global image_width
	global image_height
	
	#print(str(w))
	image_width = w
	image_height = h
	
	print("Python image dimension: " + str(image_width) + " " + str(image_height) + "\n")


def run_pedestrian_tracker(carmen_image, string_predictions):
	global image_width
	global image_height
	
	print(string_predictions)
	
	#image = carmen_image.view(np.uint8).reshape((image_height, image_width, 3))[:,:,::-1]
	#print(carmen_image.shape)
	#carmen_image = carmen_image[:,:,::-1]
	carmen_image = cv2.cvtColor(carmen_image, cv2.COLOR_BGR2RGB)
	# image = np.array([ord(c) for c in carmen_image][:(image_height * image_width * 3)]).astype(np.uint8).reshape((image_height, image_width, 3))[:,:,::-1]
	#print(image[0,0])
	#print(len(carmen_image))
	cv2.imshow("Teste python", carmen_image)
	cv2.waitKey(10)
	#plt.show()
	#print(image.shape)