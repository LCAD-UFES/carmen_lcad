include ../../Makefile.conf

# Migração Ubuntu 26.04: o módulo linkava contra o OpenCV mas nunca pedia os -I dele.
# No Ubuntu 20.04 os headers ficavam em /usr/include/opencv2 (path padrão); o pacote do
# 26.04 põe tudo em /usr/include/opencv4, que exige -I explícito.
IFLAGS += `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv`

MODULE_NAME = libCamShift
MODULE_COMMENT = CamShift Library

LINK = g++

IFLAGS += -I./
IFLAGS += -I$(OPENCV_HOME)/include
LFLAGS += -L$(OPENCV_HOME)/lib
LFLAGS += `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv`
LFLAGS += -lparam_interface -lipc -lglobal -lbumblebee_basic_interface

SOURCES = ./vertRetangulo.cpp \
    ./visualControl.cpp \
	./kalmanFilter.cpp \
	./mainCamShift.cpp \
	../visual_tracker_util.c

TARGETS = libCamShift.a camshift

libCamShift.a: ./vertRetangulo.o \
	./kalmanFilter.o \
    ./visualControl.o

camshift: mainCamShift.o libCamShift.a ../visual_tracker_util.o

# rules

include ../../Makefile.rules