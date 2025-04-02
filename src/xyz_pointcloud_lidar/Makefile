include ../Makefile.conf

MODULE_NAME = XYZ POINTCLOUD LIDAR
MODULE_COMMENT = "XYZ Pointcloud Lidar"

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol -Wno-switch
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcontrol


# ROS
IFLAGS += -I/opt/ros/foxy/include
LFLAGS += -L/opt/ros/foxy/lib

LFLAGS += -lglobal -lipc -lparam_interface -pthread

CXXFLAGS += -std=c++14  

SOURCES = xyz_pointcloud_lidar.cpp xyz_pointcloud_lidar_interface.cpp
TARGETS = xyz_pointcloud_lidar libxyz_pointcloud_lidar_interface.a
PUBLIC_BINARIES = xyz_pointcloud_lidar
PUBLIC_INCLUDES = xyz_pointcloud_lidar_interface.h xyz_pointcloud_lidar_messages.h
PUBLIC_LIBRARIES = libxyz_pointcloud_lidar_interface.a 

libxyz_pointcloud_lidar_interface.a: xyz_pointcloud_lidar_interface.o

xyz_pointcloud_lidar: xyz_pointcloud_lidar.o xyz_pointcloud_lidar_interface.o

include ../Makefile.rules
