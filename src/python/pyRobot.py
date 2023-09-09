# file: runme.py
# This file illustrates the cross language polymorphism using directors.
import pyCarmen
from threading import Thread
from time import sleep
from sys import exit
# from scipy import *
from pyCarmen import carmen_point_t
from PIL import Image
import numpy as np

#You need to override the callback function in order
#to get the functionality of this class
class pyMessageHandler(pyCarmen.MessageHandler, Thread):
	def __init__(self):
		pyCarmen.carmen_ipc_initialize(1, [b"python"])
		pyCarmen.MessageHandler.__init__(self)

		Thread.__init__(self)
		self.laser = None

	def run(self):
		self.connect()

	def run_cb(self, my_type, msg_functor):
		msg = eval("self."+msg_functor)
		self.callback(my_type, msg)

	def callback(self, the_type, msg):
		#print the_type
		#print dir(msg)
		pass

	def connect(self):
		while(1):
			#pyCarmen.carmen_ipc_sleep(0.05)
			pyCarmen.IPC_listen(10)
		#pyCarmen.carmen_ipc_dispatch()

	def stop(self):
		pyCarmen.carmen_ipc_disconnect()
		
	def __del__(self):
		print("PyCallback.__del__()")
		# pyCarmen.MessageHandler.__del__(self)
		

class Robot(pyMessageHandler):
	
	def __init__(self):
		self.name = ""
		self.year = 2013
		self.pose = pyCarmen.carmen_point_t()
		self.gps = pyCarmen.carmen_vector_3D_t()
		self.dataset_file = open("/dados/DIARA/UFES-{0}.csv".format(self.year), "w")	
		self.dataset_file.write("year,timestamp,x,y\n")
		pyMessageHandler.__init__(self)

	def __del__(self):
		self.dataset_file.close()
		pyMessageHandler.__del__(self)
		
	def callback(self, the_type, msg):
		#print the_type
		
		if(the_type == "global_pose"):
			self.set_pose(msg.globalpos.x, msg.globalpos.y, msg.globalpos.theta)
			#print "global_pose", msg.globalpos.x, msg.globalpos.y
		
		if(the_type == "gps_xyz"): # 20Hz
			self.set_gps(msg.x, msg.y, msg.z)
			#print the_type, msg.x, msg.y, msg.z
		
		if(the_type == "stereoimage8"): # 13Hz
			self.set_image(msg["raw_left"], msg["raw_right"], msg["width"], msg["height"], msg["timestamp"])
			#print the_type, msg.timestamp

		pass

	def set_pose(self, x, y, theta):
		self.pose.x = x
		self.pose.y = y
		self.pose.theta = theta
		
	def set_gps(self, x, y, z):
		self.gps.x = x
		self.gps.y = y
		self.gps.z = z
		
	def set_image(self, left, right, width, height, timestamp):
		self.left_image = Image.frombytes(mode="RGB", size=(width,height), data=np.asarray(left,dtype=np.uint8))
		self.left_image.save("/dados/DIARA/log_%d_%lf_left.ppm" % (self.year, timestamp) )
		
		self.right_image = Image.frombytes(mode="RGB", size=(width,height), data=np.asarray(right,dtype=np.uint8))
		self.right_image.save("/dados/DIARA/log_%d_%lf_right.ppm" % (self.year, timestamp) )
		
		self.dataset_file.write("%d,%lf,%lf,%lf\n" % (self.year, timestamp, self.gps.x, self.gps.y) )
		
	def set_goal(self, x, y):
		pyCarmen.carmen_navigator_set_goal(x, y)
		
	def set_goal_name(self, name):
		pyCarmen.carmen_navigator_set_goal_place(name)
		
	def command_velocity(self, tv, rv):
		pyCarmen.carmen_robot_velocity_command(tv, rv)
		
	def command_vector(self, distance, theta):
		pyCarmen.carmen_robot_move_along_vector(distance, theta)
		
	def command_go(self):
		pyCarmen.carmen_navigator_go()
		
	def command_stop(self):
		pyCarmen.carmen_navigator_stop()
						

