include ../Makefile.conf

MODULE_NAME = Neural Car Detector
MODULE_COMMENT = Detect vehicles using convolutional neural networks.

LINK = g++

CFLAGS += -Wall
CXXFLAGS = -Wall -std=c++11 -DUSE_OPENCV

IFLAGS += `pkg-config opencv --cflags`
LFLAGS += `pkg-config opencv --libs`

IFLAGS += -I $(NVCAFFE_HOME)/include -I $(NVCAFFE_HOME)/build/src/ -I $(NVCAFFE_HOME)/3rdparty -I /usr/local/cuda/include
LFLAGS += -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lparam_interface \
		-L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lrt \
		-lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
		-lboost_thread-mt -lrt -lboost_signals -lboost_system \
		-lvelodyne_camera_calibration \
		-lcamera_boxes_to_world \
		-lDetectNet

LFLAGS += -L $(NVCAFFE_HOME)/build/lib -lcaffe-nv \
		-L /usr/local/cuda/lib64 -lglog -lprotobuf -L/usr/local/lib

SOURCES = DetectNet.cpp neural_car_detector_main.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 

TARGETS = libDetectNet.a velodyne_camera_calibration camera_boxes_to_world neural_car_detector

#rules
libDetectNet.a: DetectNet.o

neural_car_detector: neural_car_detector_main.o

velodyne_camera_calibration:
	$(MAKE) -C $(CARMEN_HOME)/src/velodyne_camera_calibration
	
camera_boxes_to_world:
	$(MAKE) -C $(CARMEN_HOME)/src/camera_boxes_to_world

include ../Makefile.rules

