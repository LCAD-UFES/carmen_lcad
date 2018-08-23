include ../Makefile.conf

MODULE_NAME = Lane Detector
MODULE_COMMENT = Detect lane using convolutional neural networks.

LINK = g++

CFLAGS += -Wall -g
CXXFLAGS += -Wno-sign-compare -DUSE_OPENCV -g 

IFLAGS += `pkg-config opencv --cflags` -std=c++11 -DNO_NVML -g
LFLAGS += `pkg-config opencv --libs` -std=c++11



LFLAGS += -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lparam_interface \
		-L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lrt \
		-lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
		-lboost_thread-mt -lrt -lboost_signals -lboost_system \
		-lmoving_objects_interface \
		-llocalize_ackerman_interface \
		-lvelodyne_camera_calibration \
		-lcamera_boxes_to_world \
		-lDbscan \
		-ldarknet
#		-lDetectNet \

LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet/lib

#LFLAGS += -L $(NVCAFFE_HOME)/build/lib -lcaffe-nv \
#		-L /usr/local/cuda/lib64 -lglog -lprotobuf -L/usr/local/lib



SOURCES = dbscan.cpp lane_detector.cpp lane_detector_main.cpp lane_detector_interface.cpp # DetectNet.cpp 
# SOURCES = dbscan.cpp neural_car_detector.cpp neural_car_detector_main.cpp

PUBLIC_INCLUDES = lane_detector_interface.h lane_detector_messages.h
PUBLIC_LIBRARIES = liblane_detector_interface.a
PUBLIC_BINARIES = lane_detector

TARGETS = darknet velodyne_camera_calibration camera_boxes_to_world libDbscan.a liblane_detector_interface.a lane_detector  #libDetectNet.a
# TARGETS = velodyne_camera_calibration camera_boxes_to_world libDbscan.a neural_car_detector

#rules
#libDetectNet.a: DetectNet.o

libDbscan.a: dbscan.o

liblane_detector_interface.a: lane_detector_interface.o

lane_detector: lane_detector_main.o lane_detector.o lane_detector_interface.o

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

