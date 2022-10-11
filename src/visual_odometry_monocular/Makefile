include ../Makefile.conf

MODULE_NAME = CAMERA ODOMETRY
MODULE_COMMENT = Camera Odometry

LINK = g++
CXXFLAGS += -std=c++17

IFLAGS += -I/usr/include/eigen3

LFLAGS +=

LFLAGS += -lglobal -lipc -lparam_interface -lbumblebee_basic_interface -lcamera_interface -lm `pkg-config --cflags opencv` `pkg-config --libs opencv`

SOURCES = camera_odometry_main.cpp

PUBLIC_BINARIES = camera_odometry BigFloat.cc
PUBLIC_LIBRARIES =

TARGETS = camera_odometry

camera_odometry: camera_odometry_main.o BigFloat.o

include ../Makefile.rules