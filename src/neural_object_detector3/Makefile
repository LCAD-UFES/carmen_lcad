include ../Makefile.conf

MODULE_NAME = Neural Object Detector
MODULE_COMMENT = Detect vehicles using convolutional neural networks.

LINK = g++
CXXFLAGS = -std=c++11

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ 

LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet2/lib -L$(CARMEN_HOME)/sharedlib/libtf/src #-L$(CARMEN_HOME)/src/bumblebee_basic

LFLAGS += -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lparam_interface -llocalize_ackerman_interface \
		  -ldarknet2 `pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics \
		  -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system
#-lvelodyne_camera_calibration

SOURCES = neural_object_detector_main.cpp

TARGETS = neural_object_detector

neural_object_detector: neural_object_detector_main.o

include ../Makefile.rules