include ../Makefile.conf

# Migração Ubuntu 26.04: o módulo linkava contra o OpenCV mas nunca pedia os -I dele.
# No Ubuntu 20.04 os headers ficavam em /usr/include/opencv2 (path padrão); o pacote do
# 26.04 põe tudo em /usr/include/opencv4, que exige -I explícito.
IFLAGS += `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv`

MODULE_NAME = Camera boxes to world
MODULE_COMMENT = Find global coordinates of bounding boxes in camera image.

LINK = g++

CFLAGS += -Wall
CXXFLAGS += -Wall -std=c++14 -Wno-parentheses -Wno-deprecated-copy -Wno-implicit-fallthrough -Wno-deprecated-declarations

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I/usr/include/bullet/ -DUSE_OPENCV -DNO_NVML -DBOOST_SIGNALS_NO_DEPRECATION_WARNING
LFLAGS += -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lparam_interface `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv` \
          -lvelodyne_camera_calibration \
          -lcamera_boxes_to_world

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src 
LFLAGS += -ltf -lpthread -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread -lrt -lopencv_imgcodecs

SOURCES = camera_boxes_to_world.cpp show_boxes.cpp

PUBLIC_INCLUDES = camera_boxes_to_world.h
PUBLIC_LIBRARIES = libcamera_boxes_to_world.a
PUBLIC_BINARIES = 

TARGETS = libcamera_boxes_to_world.a show_boxes

libcamera_boxes_to_world.a: camera_boxes_to_world.o

show_boxes: show_boxes.o

include ../Makefile.rules
