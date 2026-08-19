include ../Makefile.conf

# Migração Ubuntu 26.04: faltavam os -I do OpenCV (headers agora em /usr/include/opencv4).
IFLAGS += `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv`

MODULE_NAME = PI CAMERA
MODULE_COMMENT = Raspberry Pi Camera Driver Module

LINK = g++
CXXFLAGS += -std=c++0x

IFLAGS +=
LFLAGS +=

LFLAGS += -lbumblebee_basic_interface -lcamera_interface -lparam_interface -lipc -lglobal -lm `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv` `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv` 

SOURCES = pi_camera_client_driver.cpp

PUBLIC_BINARIES = pi_camera_client_driver
PUBLIC_LIBRARIES =

TARGETS = pi_camera_client_driver

pi_camera_client_driver: pi_camera_client_driver.o

include ../Makefile.rules
