import os

def activate_virtual_environment(environment_root):
	print("""Configures the virtual environment starting at ``environment_root``.\n\n""")
	activate_script = os.path.join(
        environment_root, 'bin', 'activate_this.py')
	print("activate_script = " + activate_script)
	# execfile(activate_script, {'__file__': activate_script})
	# exec(compile(open(activate_script).read(), activate_script, 'exec'))
	exec(compile(open(activate_script, "rb").read(), activate_script, 'exec'), dict(__file__=activate_script))

	print("venv activated")

carmen_home = os.getenv("CARMEN_HOME")
virtualenv_root = carmen_home + "/src/neural_object_tracker/center_track_venv"
print("Activating Venv = " + virtualenv_root)
activate_virtual_environment(virtualenv_root)

from detector import Detector
from opts import opts

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
    #Each ret will be a list dict: [{'bbox': [x1, y1, x2, y2], 'tracking_id': id, ...}]
    ret = detector.run(img)['results']
    return ret