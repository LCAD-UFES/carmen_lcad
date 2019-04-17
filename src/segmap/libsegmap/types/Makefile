
include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = LIB_SEG_MAP TYPES
MODULE_COMMENT = Types used by segmap library

LINK = g++ 

CXXFLAGS += -Wno-deprecated -Wno-write-strings -Wno-unused-result -Wno-sign-compare -std=c++11 
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11

IFLAGS += -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ -I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`
LFLAGS += -L /usr/local/lib -lpcl_common -lpcl_io -lpcl_visualization

SOURCES = lidar_shot.cpp odom_calib_data.cpp synchronized_data_package.cpp \
	segmap_pose2d.cpp segmap_conversions.cpp
PUBLIC_INCLUDES = lidar_shot.h odom_calib_data.h synchronized_data_package.h \
	segmap_pose2d.h segmap_definitions.h segmap_conversions.h segmap_colormaps.h
PUBLIC_LIBRARIES = libsegmap_types.a
PUBLIC_BINARIES = 
TARGETS = libsegmap_types.a

libsegmap_types.a: lidar_shot.o odom_calib_data.o synchronized_data_package.o \
	segmap_pose2d.o segmap_conversions.o

include $(CARMEN_HOME)/src/Makefile.rules
