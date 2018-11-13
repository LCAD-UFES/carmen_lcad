include ../Makefile.conf

MODULE_NAME = Neural Object Detector
MODULE_COMMENT = Detect vehicles using convolutional neural networks.

LINK = g++
CXXFLAGS = -std=c++11

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -DUSE_OPENCV -DNO_NVML
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system
LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet2/lib

LFLAGS += -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lparam_interface -ltf -lrt -lLinearMath -lboost_thread-mt \
          -lboost_signals -lboost_system -lmoving_objects_interface -llocalize_ackerman_interface \
          -ldarknet2 `pkg-config opencv --libs` `pkg-config --cflags opencv` `pkg-config --libs opencv` -lpthread


SOURCES = neural_object_detector_main_test.cpp #neural_object_detector_main.cpp

TARGETS = neural_object_detector

neural_object_detector: neural_object_detector_main_test.o #neural_object_detector_main.o

include ../Makefile.rules