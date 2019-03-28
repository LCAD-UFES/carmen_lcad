
include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = EXAMPLE - VELODYNE TO PCL
MODULE_COMMENT = Example of how to create a pcl pointcloud from a velodyne file

LINK = g++ 

CXXFLAGS += -Wno-deprecated -Wno-write-strings -Wno-unused-result -Wno-sign-compare -std=c++11 
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11

IFLAGS += -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ -I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`
LFLAGS += -L /usr/local/lib -lpcl_common -lpcl_io -lpcl_visualization
LFLAGS += -L /usr/lib/x86_64-linux-gnu/ -lboost_system
LFLAGS += -L /usr/lib -lvtkIO -lvtkFiltering -lvtkCommon -lvtkRendering -lvtkGraphics

SOURCES = velodyne2pcl_main.cpp
TARGETS = velodyne2pcl_main

velodyne2pcl_main: velodyne2pcl_main.o

include $(CARMEN_HOME)/src/Makefile.rules

