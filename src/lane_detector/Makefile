include ../Makefile.conf

MODULE_NAME = Lane Detector
MODULE_COMMENT = Detect lane using convolutional neural networks.

LINK = g++

CFLAGS += -Wall -g
CXXFLAGS += -Wno-sign-compare -DUSE_OPENCV -g  -ltf

IFLAGS += `pkg-config opencv4 --cflags 2>/dev/null || pkg-config opencv --cflags` -std=c++14 -DNO_NVML -g
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I/usr/include/bullet/ -DUSE_OPENCV -DNO_NVML
IFLAGS += -I$(CARMEN_HOME)/sharedlib/darknet2
LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet2/lib
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread -lrt

LFLAGS += -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lparam_interface -ltf -lrt -lLinearMath -lboost_thread \
          -lmoving_objects_interface -llocalize_ackerman_interface -lvelodyne_camera_calibration \
          -lcamera_boxes_to_world -ldarknet2 `pkg-config opencv4 --libs 2>/dev/null || pkg-config opencv --libs` `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv` `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv` `pkg-config --libs python3-embed 2>/dev/null || python3-config --embed --libs`

LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet2/lib

#LFLAGS += -L $(NVCAFFE_HOME)/build/lib -lcaffe-nv \
#		-L /usr/local/cuda/lib64 -lglog -lprotobuf -L/usr/local/lib

SOURCES = lane_detector.cpp lane_detector_main.cpp lane_detector_interface.cpp #velodyne_camera_calibration.cpp # DetectNet.cpp 
# SOURCES = dbscan.cpp neural_car_detector.cpp neural_car_detector_main.cpp

PUBLIC_INCLUDES = lane_detector_interface.h lane_detector_messages.h
PUBLIC_LIBRARIES = liblane_detector_interface.a
PUBLIC_BINARIES = lane_detector

TARGETS = darknet velodyne_camera_calibration camera_boxes_to_world liblane_detector_interface.a lane_detector  #libDetectNet.a
# TARGETS = velodyne_camera_calibration camera_boxes_to_world libDbscan.a neural_car_detector

#rules
#libDetectNet.a: DetectNet.o

liblane_detector_interface.a: lane_detector_interface.o

lane_detector: lane_detector_main.o lane_detector.o lane_detector_interface.o velodyne_camera_calibration.o \
		| darknet velodyne_camera_calibration camera_boxes_to_world

darknet:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/darknet2
	
velodyne_camera_calibration:
	$(MAKE) -C $(CARMEN_HOME)/src/velodyne_camera_calibration
	
camera_boxes_to_world:
	$(MAKE) -C $(CARMEN_HOME)/src/camera_boxes_to_world

clean: clean_velodyne_camera_calibration clean_camera_boxes_to_world #clean_darknet

clean_velodyne_camera_calibration:
	$(MAKE) clean -C $(CARMEN_HOME)/src/velodyne_camera_calibration
	
clean_camera_boxes_to_world:
	$(MAKE) clean -C $(CARMEN_HOME)/src/camera_boxes_to_world
	
#clean_darknet:
	#$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/darknet
	
include ../Makefile.rules

