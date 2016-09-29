include ../Makefile.conf

# Module name and description
MODULE_NAME = MOVING_OBJECTS_SIMULATOR
MODULE_COMMENT = Moving objects simulator

LINK = g++

# Application specific include directories.
IFLAGS += -I/usr/include/eigen3 -I/usr/include/pcl-1.7/  -I/usr/include/vtk-5.8/
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -I$(CARMEN_HOME)/sharedlib/gl_plotter 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -L$(CARMEN_HOME)/sharedlib/gl_plotter

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lipc -lglobal \
-llocalize_ackerman_interface -lmoving_objects_interface \

# C++11 flag
CFLAGS += -Wall

# Source code files (.c, .cpp) 
SOURCES = moving_objects_simulator_main.cpp moving_objects_simulator.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = 

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = 

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = moving_objects_simulator

TARGETS = moving_objects_simulator
PUBLIC_LIBRARIES_SO = 

moving_objects_simulator:	moving_objects_simulator_main.o moving_objects_simulator.o
# rules

include ../Makefile.rules

