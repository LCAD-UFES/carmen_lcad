include ../Makefile.conf

# Module name and description
MODULE_NAME = VISUAL_CAR_TRACKING
MODULE_COMMENT = Visual car tracking

LINK = g++

# Application specific include directories.
IFLAGS += -I/usr/local/include -I./
LFLAGS += -L/usr/local/lib
LFLAGS += `pkg-config --libs opencv` 

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lipc -pthread -lglobal \
-llocalize_ackerman_interface -lmoving_objects_interface \
-lbumblebee_basic_interface

# C++11 flag
CFLAGS += -Wall

# Source code files (.c, .cpp) 
SOURCES = visual_car_tracking_main.cpp visual_car_tracking.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = visual_car_tracking.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = 

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = visual_car_tracking

TARGETS = visual_car_tracking
PUBLIC_LIBRARIES_SO = 

visual_car_tracking:	visual_car_tracking_main.o visual_car_tracking.o
# rules

include ../Makefile.rules