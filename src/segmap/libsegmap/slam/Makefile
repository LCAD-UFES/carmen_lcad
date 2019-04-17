
include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = LIB_SEG_MAP SLAM
MODULE_COMMENT = Classes and functions related to slam

LINK = g++ 

CXXFLAGS += -Wno-deprecated -Wno-write-strings -Wno-unused-result -Wno-sign-compare -std=c++11 
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11

IFLAGS += -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ \
	-I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`

SOURCES = segmap_grid_map.cpp segmap_particle_filter.cpp segmap_preproc.cpp
PUBLIC_INCLUDES = segmap_grid_map.h segmap_particle_filter.h segmap_preproc.h
PUBLIC_LIBRARIES = libsegmap_slam.a
PUBLIC_BINARIES = 
TARGETS = libsegmap_slam.a

libsegmap_slam.a: segmap_grid_map.o segmap_particle_filter.o segmap_preproc.o

include $(CARMEN_HOME)/src/Makefile.rules
