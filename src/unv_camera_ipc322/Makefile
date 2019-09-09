include ../Makefile.conf

MODULE_NAME = UNV CAMERA
MODULE_COMMENT = UNV Camera Driver Module IPC322ER3-DVPF28

LINK = g++
CXXFLAGS += -std=c++0x

IFLAGS +=
LFLAGS +=

LFLAGS += -lbumblebee_basic_interface -lcamera_interface -lparam_interface -lipc -lglobal -lm `pkg-config --cflags opencv` `pkg-config --libs opencv` 

SOURCES = unv_camera_driver.cpp

PUBLIC_BINARIES = unv_camera_driver
PUBLIC_LIBRARIES =

TARGETS = unv_camera_driver

unv_camera_driver: unv_camera_driver.o

include ../Makefile.rules
