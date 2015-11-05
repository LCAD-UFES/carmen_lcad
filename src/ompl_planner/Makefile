include ../Makefile.conf

# Module name and description
MODULE_NAME = OMPL_PACKAGE
MODULE_COMMENT = Open Motion Planning

LINK = g++

# Application specific include directories.
IFLAGS += -I/usr/include/boost
LFLAGS += -lboost_filesystem -lboost_system -lboost_thread -lboost_system -lboost_date_time -lboost_program_options -lboost_serialization 
IFLAGS += -I/usr/local/include
LFLAGS += -L/usr/local/lib -lompl

#DO NOT USE: MAY CONFLICT WITH ROS OMPL
#IFLAGS += `pkg-config --cflags ompl` 
#LFLAGS += `pkg-config --libs ompl`

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lglobal -lipc
LFLAGS += -lmap_server_interface -llocalize_ackerman_interface -lbehavior_selector_interface
CFLAGS += -Wall
# Source code files (.c, .cpp) 
SOURCES = ompl_planner_main.cpp ompl_planner_base.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = 

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = 

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = ompl_planner

TARGETS = ompl_planner
PUBLIC_LIBRARIES_SO = 

ifndef NO_PYTHON
TARGETS += 
endif

ompl_planner: ompl_planner_main.o ompl_planner_base.o

# rules

include ../Makefile.rules
