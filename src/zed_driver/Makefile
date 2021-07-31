
include ../Makefile.conf

MODULE_NAME = ZED_CAMERA
MODULE_COMMENT = Zed Camera Driver

LINK = g++
CXXFLAGS = -std=c++11 -Wno-cpp

IFLAGS += -std=c++11 -I/usr/local/zed/include -I/usr/local/cuda/include

LFLAGS += -lparam_interface -lglobal -lipc -lbumblebee_basic_interface `pkg-config --libs opencv` -L/usr/local/zed/lib -lsl_zed -L/usr/local/cuda/lib64

SOURCES = zed_driver_main.cpp

PUBLIC_BINARIES = zed_driver

TARGETS = zed_driver

zed_driver: zed_driver_main.o

include ../Makefile.rules

