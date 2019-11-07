import os

def activate_virtual_environment(environment_root):
    """Configures the virtual environment starting at ``environment_root``."""
    activate_script = os.path.join(
        environment_root, 'bin', 'activate_this.py')
    execfile(activate_script, {'__file__': activate_script})

carmen_home = os.getenv("CARMEN_HOME")
virtualenv_root = carmen_home + "/src/neural_object_detector3/nod_venv2"
print("Activating Venv")
activate_virtual_environment(virtualenv_root)

import cv2
import time
import logging
import numpy as np
from scipy.misc import imread
from tracker.mot_tracker import OnlineTracker

from utils import visualization as vis
from utils.log import logger
from utils.timer import Timer

image_width = 0
image_height = 0
tracker = OnlineTracker()

def set_image_settings(w, h):
	global image_width
	global image_height
	global tracker	

	image_width = w
	image_height = h
	
	print("Python image dimension: " + str(image_width) + " " + str(image_height) + "\n")
	
	logger.setLevel(logging.INFO)
	tracker = OnlineTracker(min_cls_score=0.1, min_ap_dist=0.8, max_time_lost=30)	
			


def run_pedestrian_tracker(carmen_image, det_tlwhs):
	global image_width
	global image_height
	global tracker
	
	#image = carmen_image.view(np.uint8).reshape((image_height, image_width, 3))[:,:,::-1]
	#print(carmen_image.shape)
	#carmen_image = carmen_image[:,:,::-1]
	#carmen_image = cv2.cvtColor(carmen_image, cv2.COLOR_BGR2RGB)
	#image = np.array([ord(c) for c in carmen_image][:(image_height * image_width * 3)]).astype(np.uint8).reshape((image_height, image_width, 3))[:,:,::-1]
	#print(image[0,0])
	#print(len(carmen_image))
	#cv2.imshow("Teste python", carmen_image)
	#cv2.waitKey(10)
	#plt.show()
	#print(image.shape)

	online_targets = tracker.update(carmen_image, det_tlwhs, None)
	online_tlwhs = []
	online_ids = []
	
	for t in online_targets:
		online_tlwhs.append(t.tlwh)
		online_ids.append(t.track_id)


	# Image Visualization
        #online_im = vis.plot_tracking(carmen_image, online_tlwhs, online_ids)
        #cv2.imshow('online_tracker', online_im)

        #cv2.waitKey(0)
        # key = chr(key % 128).lower()
        # if key == 'q':
        #     exit(0)
        # elif key == 'p':
        #     cv2.waitKey(0)
        # elif key == 'a':
        #     wait_time = int(not wait_time)

	#print(online_ids)
	#print(online_tlwhs)
	if (len(online_ids)>0):
		online_result = np.concatenate((online_tlwhs, np.array([online_ids]).T), axis=1)
		online_result = np.concatenate(([ len(online_ids) ], online_result.flatten()), axis=0).astype(np.int16)
	else:
		online_result = np.array(0).astype(np.int16)

	#print online_result
	return online_result
