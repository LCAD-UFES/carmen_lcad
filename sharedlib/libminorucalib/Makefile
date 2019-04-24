include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = LIBMINORUCALIB
MODULE_COMMENT = Lib Minoru Calib

LINK = g++

CXXFLAGS += -O3 -fopenmp
LFLAGS += -fopenmp

CFLAGS += `pkg-config opencv --cflags`
LFLAGS += `pkg-config opencv --libs` 

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libminoru
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libminoru -lminoru

SOURCES = camcalib.cpp rectify.cpp cvex/cvex.cpp cvex/CvexCameraCalibration.cpp cvex/CvexStereoCameraCalibration.cpp
TARGETS = libminorucalib.a

ifndef NO_PYTHON
TARGETS += libminorucalib.so.1
endif

libminorucalib.a: camcalib.o rectify.o cvex/cvex.o cvex/CvexCameraCalibration.o cvex/CvexStereoCameraCalibration.o

libminorucalib.so.1: camcalib.o rectify.o cvex/cvex.o cvex/CvexCameraCalibration.o cvex/CvexStereoCameraCalibration.o

# rules
include $(CARMEN_HOME)/src/Makefile.rules
