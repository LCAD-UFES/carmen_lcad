include $(CARMEN_HOME)/src/Makefile.conf

# Module name and description
MODULE_NAME = MINORU_CALIB_PACKAGE
MODULE_COMMENT = Minoru Calib Package 

LINK = g++

CXXFLAGS += -Wall -fopenmp
LFLAGS += -fopenmp

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libelas
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libelas

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libminoru
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libminoru

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libminorucalib
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libminorucalib

IFLAGS += 
LFLAGS += -lminoru -lminorucalib -lelas

IFLAGS += `pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv`

# Source code files (.c, .cpp) 
SOURCES = main.cpp anyoption.cpp drawing.cpp fast.cpp gridmap3d.cpp linefit.cpp motionmodel.cpp pointcloud.cpp polynomial.cpp stereo.cpp stereodense.cpp

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = minorucalib

TARGETS = libminoru.a libminorucalib.a minorucalib

minorucalib: main.o anyoption.o drawing.o fast.o gridmap3d.o linefit.o motionmodel.o pointcloud.o polynomial.o stereo.o stereodense.o 

libminoru.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libminoru 

libminorucalib.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libminorucalib 

# rules

include $(CARMEN_HOME)/src/Makefile.rules
