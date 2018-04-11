include ../Makefile.conf

MODULE_NAME = Neural Object Detector
MODULE_COMMENT = Detect vehicles using convolutional neural networks.

LINK = g++

CFLAGS += -Wall
CXXFLAGS = -Wno-sign-compare -DUSE_OPENCV `pkg-config opencv --cflags` -std=c++11 

IFLAGS += -DNO_NVML 
LFLAGS += `pkg-config opencv --libs`
LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet/lib -L$(CARMEN_HOME)/sharedlib/libtf/src

LFLAGS += -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lparam_interface \
		-ltf -lrt -lBulletCollision -lBulletDynamics \
		-lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system \
		-lmoving_objects_interface -llocalize_ackerman_interface -lvelodyne_camera_calibration \
		-lcamera_boxes_to_world -lDbscan -ldarknet

SOURCES = dbscan.cpp neural_object_detector.cpp neural_object_detector_main.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = neural_object_detector

TARGETS = darknet velodyne_camera_calibration camera_boxes_to_world libDbscan.a neural_object_detector

libDbscan.a: dbscan.o

neural_object_detector: neural_object_detector_main.o neural_object_detector.o

darknet:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/darknet
	
velodyne_camera_calibration:
	$(MAKE) -C $(CARMEN_HOME)/src/velodyne_camera_calibration
	
camera_boxes_to_world:
	$(MAKE) -C $(CARMEN_HOME)/src/camera_boxes_to_world

clean: clean_velodyne_camera_calibration clean_camera_boxes_to_world clean_darknet

clean_velodyne_camera_calibration:
	$(MAKE) clean -C $(CARMEN_HOME)/src/velodyne_camera_calibration
	
clean_camera_boxes_to_world:
	$(MAKE) clean -C $(CARMEN_HOME)/src/camera_boxes_to_world
	
clean_darknet:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/darknet
	
include ../Makefile.rules

