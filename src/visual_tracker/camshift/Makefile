include ../../Makefile.conf

MODULE_NAME = libCamShift
MODULE_COMMENT = CamShift Library

LINK = g++

IFLAGS += -I./
IFLAGS += -I$(OPENCV_HOME)/include
LFLAGS += -L$(OPENCV_HOME)/lib
LFLAGS += `pkg-config --libs opencv`
LFLAGS += -lparam_interface -lipc -lglobal -lbumblebee_basic_interface

SOURCES = ./vertRetangulo.cpp \
    ./visualControl.cpp \
	./kalmanFilter.cpp \
	./mainCamShift.cpp \
	../visual_tracker_util.c

TARGETS = libCamShift.a camshift

libCamShift.a: ./vertRetangulo.o \
	./kalmanFilter.o \
    ./visualControl.o

camshift: mainCamShift.o libCamShift.a ../visual_tracker_util.o

# rules

include ../../Makefile.rules