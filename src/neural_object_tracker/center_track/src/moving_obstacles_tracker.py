# import _init_paths
import os
import sys
import cv2
import time
import numpy as np
if not hasattr(sys, 'argv'): #https://github.com/googleapis/oauth2client/issues/642
    sys.argv  = ['']

carmen_home = os.getenv("CARMEN_HOME")
virtualenv_root = carmen_home + "/src/neural_object_tracker/center_track_venv"
CENTERTRACK_PATH = carmen_home + "/src/neural_object_tracker/center_track/src/lib"
sys.path.insert(0, CENTERTRACK_PATH)

def activate_virtual_environment(environment_root):
    # print("""Configures the virtual environment starting at ``environment_root``.\n\n""")
    print("Activating Venv = " + environment_root)
    activate_script = os.path.join(
        environment_root, 'bin', 'activate_this.py')
    print("activate_script = " + activate_script)
    # execfile(activate_script, {'__file__': activate_script})
    # exec(compile(open(activate_script).read(), activate_script, 'exec'))
    exec(compile(open(activate_script, "rb").read(), activate_script, 'exec'), dict(__file__=activate_script))

    print("venv activated")


# activate_virtual_environment(virtualenv_root)

from opts import opts
from detector import Detector
MODEL_PATH = carmen_home + "/src/neural_object_tracker/center_track/models/coco_tracking.pth"
DATASET = 'coco'
TASK = 'tracking' # or 'tracking,multi_pose' for pose tracking and 'tracking,ddd' for monocular 3d tracking
opt = opts().init('{} --load_model {}'.format(TASK, MODEL_PATH).split(' '))
detector = Detector(opt)

def set_image_settings(w, h):
    global image_width
    global image_height
    global tracker	

    image_width = w
    image_height = h
    
    print("Python image dimension: " + str(image_width) + " " + str(image_height) + "\n")


def run_moving_obstacles_tracker(carmen_image):
    # carmen_image = carmen_image[:,:,::-1]
    # carmen_image = cv2.cvtColor(carmen_image, cv2.COLOR_BGR2RGB)
    # # print(len(carmen_image))
    # cv2.imshow("Teste python", carmen_image)
    # cv2.waitKey(1)
    # #Each ret will be a list dict: [{'bbox': [x1, y1, x2, y2], 'tracking_id': id, ...}]
    start_time = time.time() # start time of the loop
    ret = detector.run(carmen_image)['results']
    # print("FPS: ", 1.0 / (time.time() - start_time)) # FPS = 1 / time to process loop
    dets = []
    dets.append(int(len(ret)))
    for r in ret:
        dets.append(float(r['score']))
        dets.append(int(r['class']))
        dets.append(float(r['bbox'][0]))
        dets.append(float(r['bbox'][1]))
        dets.append(float(r['bbox'][2] - r['bbox'][0]))
        dets.append(float(r['bbox'][3] - r['bbox'][1]))
        dets.append(int(r['tracking_id']))
    #[num_dets, score_0, class_0, bbox_x_0, bbox_y_0, bbox_w_0, bbox_h_0, track_id_0, ..., score_n, class_n, bbox_x_n, bbox_y_n, bbox_w_n, bbox_h_n, track_id_n]
    # print(dets)
    dets = np.array(dets)
    return (dets.astype(np.float64))