
include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = LIB_SEG_MAP READERS
MODULE_COMMENT = Functions for reading sensors data 

LINK = g++ 

CXXFLAGS += -Wno-deprecated -Wno-write-strings -Wno-unused-result -Wno-sign-compare -std=c++11 
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11

IFLAGS += -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ -I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`

SOURCES = carmen_image_reader.cpp carmen_lidar_reader.cpp carmen_semantic_segmentation_reader.cpp # segmap_pose_reader.cpp
PUBLIC_INCLUDES = carmen_image_reader.h carmen_lidar_reader.h carmen_semantic_segmentation_reader.h # segmap_pose_reader.h
PUBLIC_LIBRARIES = libsegmap_readers.a
PUBLIC_BINARIES = 
TARGETS = libsegmap_readers.a

libsegmap_readers.a: carmen_image_reader.o carmen_lidar_reader.o carmen_semantic_segmentation_reader.o # segmap_pose_reader.o
 

include $(CARMEN_HOME)/src/Makefile.rules
