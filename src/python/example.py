from pyRobot import *
from math import *
from time import sleep

def save_dataset():
	# Create an callback
	robot = Robot().__disown__()

	# Create callers
	#loc = pyCarmen.global_pose(robot)
	#odo = pyCarmen.odometry(robot)
	gps = pyCarmen.gps_xyz(robot)
	stereo = pyCarmen.stereoimage8(robot)
	
	# Dispatch and don't return
	robot.connect()

if __name__ == "__main__":
	save_dataset()