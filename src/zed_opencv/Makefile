include ../Makefile.conf

# Module name and description
MODULE_NAME = ZED OPENCV
MODULE_COMMENT = Zed Camera Module OpenCV

LINK = g++

# Application specific include directories.
IFLAGS += -I/usr/include/eigen3

# Required default libraries to comunicate with Carmen
LFLAGS += -lparam_interface -lglobal -lipc -lbumblebee_basic_interface -lstereo_interface

# Opencv links
LFLAGS += `pkg-config --libs opencv`

CFLAGS   += -Wall `pkg-config --cflags opencv` -O3
CXXFLAGS += -Wno-switch -O3 -std=c++11 -std=c++11 -O3 -Wno-reorder -Wno-unused-parameter -Wno-ignored-qualifiers -Wno-unused-but-set-variable

# Source code files (.c, .cpp)
SOURCES = zed_opencv.cpp

# Public headers, linked to 'carmen/include/carmen/'.
# PUBLIC_INCLUDES =

# Public libraries, linked to 'carmen/lib'.
#PUBLIC_LIBRARIES =

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = zed_opencv

TARGETS = zed_opencv

all: zed_opencv

zed_opencv: zed_opencv.o

# rules
include ../Makefile.rules

