
include ../Makefile.conf

# Module name and description
MODULE_NAME = realsense_CAMERA
MODULE_COMMENT = realsense Camera Module

LINK = g++

# Required default libraries to comunicate with Carmen Core.
#LFLAGS += `pkg-config --libs opencv` -lparam_interface -lglobal -lbumblebee_basic_interface -lipc
LFLAGS += 	-lparam_interface -lglobal -lbumblebee_basic_interface -llaser_ldmrs_interface -lipc -lrealsense2\
			-lstereo_interface -lstereo_velodyne -lstereo_velodyne_interface `pkg-config --libs opencv` -std=c++11 

CFLAGS += -Wall -std=c++11 #`pkg-config --cflags opencv`
CXXFLAGS += -Wall -std=c++11 #`pkg-config --cflags opencv` -std=c++11

SOURCES = realsense_camera_sensor_main.cpp 
PUBLIC_BINARIES = realsense_camera_main
TARGETS = realsense_camera_main

realsense_camera_main: realsense_camera_sensor_main.o

include ../Makefile.rules

