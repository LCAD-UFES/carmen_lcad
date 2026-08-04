include ../Makefile.conf

# Module name and description
MODULE_NAME = MOVING_OBJECTS_SIMULATOR
MODULE_COMMENT = Moving objects simulator

LINK = g++

# Application specific include directories.
PCL_CFLAGS := $(shell pkg-config --cflags pcl_common pcl_io pcl_visualization pcl_registration pcl_kdtree pcl_features pcl_search pcl_octree pcl_sample_consensus pcl_filters pcl_surface pcl_segmentation 2>/dev/null)
PCL_LIBS := $(shell pkg-config --libs pcl_common pcl_io pcl_visualization pcl_registration pcl_kdtree pcl_features pcl_search pcl_octree pcl_sample_consensus pcl_filters pcl_surface pcl_segmentation 2>/dev/null)
IFLAGS += -I/usr/include/eigen3 $(PCL_CFLAGS)

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -I$(CARMEN_HOME)/sharedlib/gl_plotter 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -L$(CARMEN_HOME)/sharedlib/gl_plotter -L/usr/local/carmen_boost/lib

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lipc -lglobal -lvelodyne_interface \
-L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lboost_thread -lrt -lboost_system -pthread \
-llocalize_ackerman_interface -lmoving_objects_interface -lmap_server_interface -lmapper_interface \
$(PCL_LIBS)

# C++11 flag
CFLAGS += -Wall
CXXFLAGS += -std=gnu++14

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

# If you set graphics support in configure.
ifndef NO_GRAPHICS
#Carmen Global Graphic Library and Gtk+2 Graphics Libraries
LFLAGS += -lglobal_graphics
IFLAGS += `pkg-config --cflags gtk+-2.0 gmodule-export-2.0`
IFLAGS += `pkg-config --cflags gtkglext-1.0`
LFLAGS += `pkg-config --libs gtk+-2.0 gmodule-export-2.0`
LFLAGS += `pkg-config --libs gtkglext-1.0`
endif

moving_objects_simulator:	moving_objects_simulator_main.o moving_objects_simulator.o
# rules

include ../Makefile.rules
