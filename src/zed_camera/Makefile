include ../Makefile.conf

# Module name and description
MODULE_NAME = ZED_CAMERA
MODULE_COMMENT = Zed Camera Module

LINK = g++

# Application specific include directories.
IFLAGS += -I/usr/local/zed/include -I/usr/include/eigen3 -I/usr/local/cuda/include

# Required default libraries to comunicate with Carmen Core.
LFLAGS += `pkg-config --libs opencv` -L/usr/local/zed/lib -L/usr/local/cuda/lib64 -lparam_interface -lipc -lglobal -lzed_camera_sensor_interface -lsl_zed -lcudpp
CFLAGS += -Wall `pkg-config --cflags opencv`
CXXFLAGS += -std=c++11
# Source code files (.c, .cpp)
SOURCES = zed_camera_sensor_main.cpp zed_camera_sensor_interface.c

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = zed_camera_sensor_messages.hpp zed_camera_sensor_interface.hpp

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libzed_camera_sensor_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = zed_camera_sensor

PUBLIC_LIBRARIES_SO = libzed_camera_sensor_interface.so

TARGETS = libzed_camera_sensor_interface.a zed_camera_sensor

libzed_camera_sensor_interface.a: zed_camera_sensor_interface.o

libzed_camera_sensor_interface.so.1: zed_camera_sensor_interface.o

zed_camera_sensor:	zed_camera_sensor_main.o

# rules

include ../Makefile.rules

