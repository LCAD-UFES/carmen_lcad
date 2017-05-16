
include ../Makefile.conf

# Module name and description
MODULE_NAME = ZED_CAMERA
MODULE_COMMENT = Zed Camera Module

LINK = g++

# Required default libraries to comunicate with Carmen Core.
LFLAGS += `pkg-config --libs opencv` -lparam_interface -lglobal -lbumblebee_basic_interface -lipc
CFLAGS += -Wall `pkg-config --cflags opencv`
CXXFLAGS += -Wall `pkg-config --cflags opencv` -std=c++11

SOURCES = zed_camera_sensor_main.cpp 
PUBLIC_BINARIES = zed_camera_main
TARGETS = zed_camera_main

zed_camera_main: zed_camera_sensor_main.o

include ../Makefile.rules

