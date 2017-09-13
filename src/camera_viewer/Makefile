include ../Makefile.conf

# Module name and description
MODULE_NAME = CAMERA_VIEWER
MODULE_COMMENT = Viewer for objects on camera image

LINK = g++

# Application specific include directories.
LFLAGS += `pkg-config --libs opencv`

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lipc -lglobal \
-llocalize_ackerman_interface -lmoving_objects_interface \
-lbumblebee_basic_interface

# C++11 flag
CFLAGS += -Wall

# Source code files (.c, .cpp)
SOURCES = camera_viewer_main.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES =

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES =

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = camera_viewer

TARGETS = camera_viewer
PUBLIC_LIBRARIES_SO =

camera_viewer:	camera_viewer_main.o
# rules

include ../Makefile.rules