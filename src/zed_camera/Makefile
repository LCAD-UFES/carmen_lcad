include ../Makefile.conf

# Module name and description
MODULE_NAME = ZED_CAMERA
MODULE_COMMENT = Zed Camera Module

LINK = g++

# Application specific include directories.
IFLAGS += -I/usr/local/zed/include -I/usr/include/eigen3 -I/usr/local/cuda/include

# Required default libraries to comunicate with Carmen Core.
LFLAGS += `pkg-config --libs opencv` -L/usr/local/zed/lib -L/usr/local/cuda/lib64 -lparam_interface -lglobal -lipc \
		-lzed_camera_sensor_interface -lsl_zed -lcudpp -lbumblebee_basic_interface -lstereo_interface
		
CFLAGS += -Wall `pkg-config --cflags opencv` -O3
CXXFLAGS += -Wno-switch -O3 -std=c++11 
# Source code files (.c, .cpp)
SOURCES = zed_camera_sensor_main.cpp zed_camera_sensor_interface.c zed_camera_sensor_viewer.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = zed_camera_sensor_messages.hpp zed_camera_sensor_interface.hpp

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libzed_camera_sensor_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = zed_camera_sensor zed_camera_sensor_viewer

PUBLIC_LIBRARIES_SO = libzed_camera_sensor_interface.so

TARGETS = libzed_camera_sensor_interface.a zed_camera_sensor zed_camera_sensor_viewer

libzed_camera_sensor_interface.a: zed_camera_sensor_interface.o

libzed_camera_sensor_interface.so.1: zed_camera_sensor_interface.o

zed_camera_sensor:	zed_camera_sensor_main.o
zed_camera_sensor_viewer: zed_camera_sensor_viewer.o
# rules

include ../Makefile.rules

