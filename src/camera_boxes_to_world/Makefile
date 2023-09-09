include ../Makefile.conf

MODULE_NAME = Camera boxes to world
MODULE_COMMENT = Find global coordinates of bounding boxes in camera image.

LINK = g++

CFLAGS += -Wall
CXXFLAGS += -Wall -std=c++11 -Wno-parentheses -Wno-deprecated-copy -Wno-implicit-fallthrough -Wno-deprecated-declarations

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I/usr/include/bullet/ -DUSE_OPENCV -DNO_NVML -DBOOST_SIGNALS_NO_DEPRECATION_WARNING
IFLAGS += -I/usr/local/carmen_boost/include
LFLAGS += -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lparam_interface `pkg-config --libs opencv` \
          -lvelodyne_camera_calibration \
          -lcamera_boxes_to_world

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L/usr/local/carmen_boost/lib 
LFLAGS += -ltf -lpthread -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system -lopencv_imgcodecs

SOURCES = camera_boxes_to_world.cpp show_boxes.cpp

PUBLIC_INCLUDES = camera_boxes_to_world.h
PUBLIC_LIBRARIES = libcamera_boxes_to_world.a
PUBLIC_BINARIES = 

TARGETS = libcamera_boxes_to_world.a show_boxes

libcamera_boxes_to_world.a: camera_boxes_to_world.o

show_boxes: show_boxes.o

include ../Makefile.rules
