include ../Makefile.conf

MODULE_NAME = CAMERA IP Driver
MODULE_COMMENT = Camera IP Driver Modules

LINK = g++
CFLAGS += -std=c11
CXXFLAGS += -std=c++11

IFLAGS += `pkg-config --cflags opencv`

#OPENCV
IFLAGS += -DUSE_OPENCV 
LFLAGS += `pkg-config opencv --libs` `pkg-config --cflags opencv` `pkg-config --libs opencv` 

LFLAGS += -lbumblebee_basic_interface -lcamera_interface -lparam_interface -lipc -lglobal -lm `pkg-config --cflags opencv` `pkg-config --libs opencv` 

SOURCES = unv_camera_driver.cpp rtsp_camera_driver.cpp

PUBLIC_BINARIES = unv_camera_driver rtsp_camera_driver 
PUBLIC_LIBRARIES =

TARGETS = unv_camera_driver rtsp_camera_driver 

unv_camera_driver: unv_camera_driver.o

rtsp_camera_driver: rtsp_camera_driver.o

include ../Makefile.rules
