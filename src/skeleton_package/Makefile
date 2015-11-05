include ../Makefile.conf

# Module name and description
MODULE_NAME = SKELETON_PACKAGE
MODULE_COMMENT = Skeleton Package Example

LINK = g++

# Application specific include directories.
IFLAGS += 

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lipc -lglobal -lskeleton_module_sensor_interface -lskeleton_module_filter_interface
CFLAGS += -Wall
# Source code files (.c, .cpp) 
SOURCES = skeleton_module_sensor_main.c skeleton_module_sensor_interface.c skeleton_module_filter_main.c skeleton_module_filter_interface.c

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = skeleton_module_sensor_messages.h skeleton_module_sensor_interface.h skeleton_module_filter_messages.h skeleton_module_panel.h skeleton_module_filter_interface.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libskeleton_module_sensor_interface.a  libskeleton_module_filter_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = skeleton_module_sensor skeleton_module_filter

TARGETS = libskeleton_module_sensor_interface.a skeleton_module_sensor libskeleton_module_filter_interface.a skeleton_module_filter
PUBLIC_LIBRARIES_SO = libskeleton_module_sensor_interface.so libskeleton_module_filter_interface.so

libskeleton_module_sensor_interface.a: skeleton_module_sensor_interface.o

libskeleton_module_sensor_interface.so.1: skeleton_module_sensor_interface.o

skeleton_module_sensor:	skeleton_module_sensor_main.o 

libskeleton_module_filter_interface.a: skeleton_module_filter_interface.o

libskeleton_module_filter_interface.so.1: skeleton_module_filter_interface.o

skeleton_module_filter:	skeleton_module_filter_main.o

# rules

include ../Makefile.rules

