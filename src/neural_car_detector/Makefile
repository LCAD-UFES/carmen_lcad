include ../Makefile.conf

MODULE_NAME = Neural Car Detector
MODULE_COMMENT = Detect vehicles using convolutional neural networks.

LINK = g++

CFLAGS += -Wall -g
CXXFLAGS = -Wno-sign-compare -DUSE_OPENCV -g 

IFLAGS += `pkg-config opencv --cflags` -std=c++11 -DNO_NVML -g
LFLAGS += `pkg-config opencv --libs` -std=c++11

ifdef NVCAFFE_HOME
IFLAGS += -I $(NVCAFFE_HOME)/include -I $(NVCAFFE_HOME)/build/src/ -I $(NVCAFFE_HOME)/3rdparty -I /usr/local/cuda/include
endif

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
LFLAGS += -L $(NVCAFFE_HOME)/build/lib \
 		-L /usr/local/cuda/lib64 -lglog -lprotobuf -L/usr/local/lib


SOURCES = dbscan.cpp neural_car_detector.cpp neural_car_detector_main.cpp # DetectNet.cpp 
# SOURCES = dbscan.cpp neural_car_detector.cpp neural_car_detector_main.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = neural_car_detector

TARGETS = darknet velodyne_camera_calibration camera_boxes_to_world libDbscan.a neural_car_detector  #libDetectNet.a
# TARGETS = velodyne_camera_calibration camera_boxes_to_world libDbscan.a neural_car_detector

#rules
#libDetectNet.a: DetectNet.o

libDbscan.a: dbscan.o

neural_car_detector: neural_car_detector_main.o neural_car_detector.o

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

