include ../../Makefile.conf

# Module name and description
MODULE_NAME = LANE_ANALYSIS
MODULE_COMMENT = Ego-Lane Analysis System - ELAS


LINK = g++
CFLAGS +=
CXXFLAGS += -Wall -std=gnu++11

LFLAGS +=  -lparam_interface -lipc -lglobal -llocalize_ackerman_interface -lmap_server_interface -lbumblebee_basic_interface 
LFLAGS += -L./ELAS/build/ -lELAS
LFLAGS += -ldarknet
# OpenCV
IFLAGS += `pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv`
LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet/lib

SOURCES			= $(wildcard *.cpp)

PUBLIC_INCLUDES = lane_analysis_interface.h lane_analysis_messages.h
PUBLIC_LIBRARIES = liblane_analysis_interface.a
PUBLIC_BINARIES = lane_analysis

#TARGETS 		= lane_analysis lane_analysis_viz
TARGETS 		= lane_analysis

#build-elas:
#	cd ELAS && cd build && cmake  -DCMAKE_BUILD_TYPE=Debug -D BUILD_CARMEN:BOOL=TRUE -D OpenCV_DIR=/usr/local/opencv-2.4.9/build -D OpenCV_FOUND=TRUE .. && make && cd ../..
build-elas:
	mkdir -p ELAS/build
	#cd ELAS && cd build && cmake -D BUILD_CARMEN:BOOL=TRUE -D OpenCV_DIR=/usr/local/opencv-2.4.9/build -D OpenCV_FOUND=TRUE .. && make && cd ../..
    #antes estava com OpenCV 2 agora está com o 3
	cd ELAS && cd build && cmake -D BUILD_CARMEN:BOOL=TRUE -D OpenCV_DIR=/opt/opencv3.1.0/opencv/build -D OpenCV_FOUND=TRUE -DCMAKE_BUILD_TYPE=Debug .. && make && cd ../..
liblane_analysis_interface.a: lane_analysis_interface.o

lane_analysis: build-elas liblane_analysis_interface.a lane_analysis.o
#lane_analysis_viz: lane_analysis_viz_main.o

clean: clean_elas
clean_elas:
	rm -r ELAS/build

# rules
include ../../Makefile.rules
