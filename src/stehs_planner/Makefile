include ../Makefile.conf

MODULE_NAME = STEHS MOTION PLANNER
MODULE_COMMENT = Space Time Exploration Guided Heuristic Search Motion Planner 

LINK = g++
CXXFLAGS += -std=c++0x

LFLAGS += -lparam_interface -lipc -lglobal
	
SOURCES = stehs_planner_main.cpp stehs_planner.cpp

PUBLIC_BINARIES = stehs_planner
PUBLIC_LIBRARIES = 

TARGETS = stehs_planner

stehs_planner: stehs_planner_main.o stehs_planner.o

include ../Makefile.rules
