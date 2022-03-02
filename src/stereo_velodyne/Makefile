include ../Makefile.conf

# Module name and description
MODULE_NAME = Stereo VELODYNE
MODULE_COMMENT = Stereo Velodyne LIDAR

LINK = g++

# Application specific include directories.
PCL_INC = $(wildcard /usr/local/include/pcl-*)
VTK_INC = $(wildcard /usr/local/include/vtk-5* /usr/include/vtk-5*)
IFLAGS += -I/usr/include/eigen3 -I $(PCL_INC) -I $(VTK_INC)

LFLAGS += -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search -lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization  -lboost_system -lboost_thread-mt -lrt -lboost_signals -lboost_system

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libvelodyne
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libvelodyne 

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lipc -pthread -lglobal -lstereo_velodyne_interface -lstereo_velodyne -lstereo_interface `pkg-config --libs opencv` 
CFLAGS += -Wall

# Source code files (.c, .cpp) 
SOURCES = stereo_velodyne.cpp stereo_velodyne_main.cpp stereo_velodyne_interface.c

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = stereo_velodyne_messages.h stereo_velodyne_interface.h stereo_velodyne.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libstereo_velodyne_interface.a libstereo_velodyne.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = stereo_velodyne

TARGETS = libstereo_velodyne_interface.a libstereo_velodyne.a stereo_velodyne
PUBLIC_LIBRARIES_SO = libstereo_velodyne_interface.so

ifndef NO_PYTHON
TARGETS += libstereo_velodyne_interface.so.1
endif

# If you set graphics support in configure (only for GTK).
ifndef NO_GRAPHICS
FLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += `pkg-config --libs gtk+-2.0`
#Source code file (.c)
TARGETS +=
SOURCES += 
endif

libstereo_velodyne.a: stereo_velodyne.o 
libstereo_velodyne_interface.a: stereo_velodyne_interface.o
libstereo_velodyne_interface.so.1:	stereo_velodyne_interface.o
stereo_velodyne: stereo_velodyne_main.o libstereo_velodyne_interface.a libstereo_velodyne.a

# rules

include ../Makefile.rules
