include ../../Makefile.conf

MODULE_NAME = Yolo Detector
MODULE_COMMENT = Detect objects and publishes the bboxes

LINK = g++
CXXFLAGS = -std=c++11

IFLAGS += -I/usr/include/python2.7

LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet3/lib 

LFLAGS += -lglobal -lipc -lbumblebee_basic_interface -lparam_interface \
		  -ldarknet `pkg-config --libs opencv` \
		  `python2.7-config --libs` \
		  -lcamera_drivers

SOURCES = yolo_detector.cpp

PUBLIC_BINARIES = yolo_detector

TARGETS = yolo_detector

yolo_detector: yolo_detector.o

include ../../Makefile.rules