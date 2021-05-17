include ../../Makefile.conf

LINK = g++

MODULE_NAME = RRT Util
MODULE_COMMENT =   
CFLAGS += -O4 #-pg
CXXFLAGS += -std=c++0x -O4 #-pg

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcarmodel
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol

SOURCES = publisher_util.cpp util.cpp obstacle_detection.cpp dijkstra.cpp ackerman.cpp lane.cpp carmen_rrt_util.cpp cost_function.cpp

PUBLIC_INCLUDES = carmen_rrt_util.h

include ../../Makefile.rules
