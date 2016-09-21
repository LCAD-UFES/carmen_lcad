include ../Makefile.conf

# Module name and description
MODULE_NAME = MOVING_OBJECTS_SIMULATOR
MODULE_COMMENT = Moving objects simulator

LINK = g++

# Application specific include directories.
IFLAGS += 

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lgrid_mapping -lparam_interface -lipc -lglobal \
-lfused_odometry_interface -llocalize_ackerman_interface \
-llocalize_ackerman_core \
-llocalize_ackerman_motion

# C++11 flag
CFLAGS += -Wall -w -msse3

# Source code files (.c, .cpp) 
SOURCES = moving_objects_simulator_main.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = 

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = 

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = moving_objects_simulator

TARGETS = moving_objects_simulator
PUBLIC_LIBRARIES_SO = 

moving_objects_simulator:	moving_objects_simulator_main.o
# rules

include ../Makefile.rules

