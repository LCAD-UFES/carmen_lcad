include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = CAMERA DRIVER
MODULE_COMMENT = Camera Module Drivers, Visualizer and Supplementary Functions

LINK = g++
CFLAGS += -Wall -g
CXXFLAGS += -std=c++11

LFLAGS += -lparam_interface -lipc -pthread -lglobal `pkg-config --libs opencv` `pkg-config --libs libavformat libavcodec libswscale libavutil`

SOURCES = camera_drivers_main.cpp camera_drivers_interface.cpp camera_drivers_process_image.cpp camera_drivers_access_image_functions.cpp camera_viewer.cpp camera_check.cpp

PUBLIC_INCLUDES_COPY = camera_drivers_interface.h camera_drivers_messages.h camera_drivers_process_image.hpp

PUBLIC_LIBRARIES_COPY = libcamera_drivers.a libcamera_drivers_process_image.a

PUBLIC_BINARIES_COPY = camera_drivers camera_viewer camera_check

TARGETS = camera_drivers camera_viewer libcamera_drivers.a libcamera_drivers_process_image.a camera_check

camera_drivers: camera_drivers_main.o camera_drivers_interface.o camera_drivers_access_image_functions.o camera_drivers_process_image.o

camera_viewer: camera_viewer.o camera_drivers_interface.o camera_drivers_process_image.o

libcamera_drivers.a: camera_drivers_interface.o

# need to include opencv_calib3d
libcamera_drivers_process_image.a: camera_drivers_process_image.o

camera_check: camera_check.o camera_drivers_access_image_functions.o camera_drivers_interface.o camera_drivers_process_image.o

include $(CARMEN_HOME)/src/Makefile.rules
