
include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = LIB_SEG_MAP SLAM HIGH LEVEL
MODULE_COMMENT = High level slam functions

LINK = g++ 
#SUBDIRS += hash_map

CXXFLAGS += -Wno-deprecated -Wno-write-strings -Wno-unused-result -Wno-sign-compare -std=c++11 -pg 
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11 -pg
LFLAGS += -pg

IFLAGS += -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ \
	-I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`

SOURCES = segmap_map_builder.cpp
PUBLIC_INCLUDES = segmap_map_builder.h 
PUBLIC_LIBRARIES = libsegmap_slam_high_level.a
PUBLIC_BINARIES = 
TARGETS = libsegmap_slam_high_level.a 

libsegmap_slam_high_level.a: segmap_map_builder.o

include $(CARMEN_HOME)/src/Makefile.rules
