
include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = LIB_SEG_MAP INITIALIZATIONS
MODULE_COMMENT = Utility functions for initializing the segmap module.

LINK = g++ 

CXXFLAGS += -Wno-deprecated -Wno-write-strings -Wno-unused-result -Wno-sign-compare -std=c++11 
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11

IFLAGS += -I /usr/include/eigen3/ -I /usr/local/include/pcl-1.8 \
	-I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`

SOURCES = segmap_args.cpp segmap_constructors.cpp
PUBLIC_INCLUDES = segmap_args.h segmap_constructors.h
PUBLIC_LIBRARIES = libsegmap_initializations.a
PUBLIC_BINARIES = 
TARGETS = libsegmap_initializations.a

libsegmap_initializations.a: segmap_args.o segmap_constructors.o

include $(CARMEN_HOME)/src/Makefile.rules
