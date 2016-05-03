include ../Makefile.conf

# Module name and description
MODULE_NAME = LANE_ANALYSIS
MODULE_COMMENT = Ego-Lane Analysis System - ELAS


LINK = g++
CXXFLAGS += -Wall -std=c++11

LFLAGS += -lparam_interface -lipc -lglobal -llocalize_ackerman_interface -lmap_server_interface -lbumblebee_basic_interface
LFLAGS += -L./ELAS/build/ -lELAS

# OpenCV
IFLAGS += `pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv`

SOURCES			= $(wildcard *.cpp)

PUBLIC_INCLUDES = $(wildcard *.h)

PUBLIC_BINARIES = lane_analysis

TARGETS 		= lane_analysis

build-elas:
	cd ELAS && cd build && cmake -D BUILD_CARMEN:BOOL=TRUE -D OpenCV_DIR=/usr/local/opencv-2.4.12.3/build .. && make && cd ../..

lane_analysis: build-elas $(patsubst %.cpp, %.o, $(SOURCES))

# rules
include ../Makefile.rules