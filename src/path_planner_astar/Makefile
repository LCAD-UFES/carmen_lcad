
include ../Makefile.conf

# Path planner A-star
MODULE_NAME = Path Planner A-star
MODULE_COMMENT = Path Planning using A-star algorithm

LINK = g++
CFLAGS += -Wall -g 
CXXFLAGS = -std=c++11

LFLAGS += -lglobal -lipc -lparam_interface

SOURCES = path_planner_astar_main.cpp 
PUBLIC_INCLUDES = path_planner_astar.h
# Public libraries, linked to 'carmen/lib'.

PUBLIC_BINARIES = path_planner_astar
TARGETS = path_planner_astar

path_planner_astar: path_planner_astar_main.o

include ../Makefile.rules

