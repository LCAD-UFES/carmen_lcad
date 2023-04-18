include ../Makefile.conf

# Module name and description
MODULE_NAME = MOVING_OBJECTS
MODULE_COMMENT = Moving Objects Point Clouds

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libvelodyne
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I/usr/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libvelodyne
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -I$(CARMEN_HOME)/sharedlib/gl_plotter 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -L$(CARMEN_HOME)/sharedlib/gl_plotter

LFLAGS += -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lrt

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libstereovgram
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libstereo
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libelas

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libstereo
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libstereovgram
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libelas
LFLAGS += -lparam_interface -lipc -lglobal -lbumblebee_basic_interface -lelas -lstereovgram -lstereo -lstdc++ -fopenmp -lm

CFLAGS += -Wall -fopenmp
CFLAGS += -w -msse3

CFLAGS += -std=c11
CXXFLAGS += -std=c++11

# Required for debugging:
#CXXFLAGS += -g -pg -Wall -fopenmp

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lgrid_mapping -lparam_interface -lipc -lglobal -lmapper_interface -lmoving_objects_interface -lmap_server_interface -lfused_odometry_interface -llocalize_ackerman_interface -llaser_interface -lrotation_geometry -lmap_io -lmap_util \
-lmap_interface -llocalize_ackerman_core -lvelodyne_interface -lstereo_velodyne_interface -lstereo_velodyne -lstereo_interface \
-ltf -lBulletDynamics -lBulletCollision -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system \
-llocalize_ackerman_motion -lprob_models -lgl_plotter -lmap_interface 

# C++11 flag
LFLAGS += -std=c++11

IFLAGS += `pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv`

PCL_INC = $(wildcard /usr/local/include/pcl-*)
VTK_INC = $(wildcard /usr/local/include/vtk-5* /usr/include/vtk-5* /usr/include/vtk-6*)
IFLAGS += -I/usr/include/eigen3 -I $(PCL_INC) -I $(VTK_INC)

#LFLAGS += -lpcl_common -lpcl_kdtree -lpcl_search -lpcl_segmentation
#LFLAGS += -lvtkRendering -lvtkCommon -lvtkFiltering -lpcl_common -lpcl_kdtree -lpcl_features -lpcl_search -lpcl_registration -lpcl_surface -lpcl_io -lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization \
#			-lpcl_segmentation
LFLAGS += -lpcl_common -lpcl_kdtree -lpcl_features -lpcl_search -lpcl_registration -lpcl_surface -lpcl_io -lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization \
			-lpcl_segmentation

# Boost flag
LFLAGS += -I/usr/local/carmen_boost/include

# Source code files (.c, .cpp) 
SOURCES = moving_objects_main.cpp moving_objects_interface.c moving_objects.cpp monte_carlo_moving_objects_tracking.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = moving_objects_types.h moving_objects_messages.h moving_objects_interface.h moving_objects.h monte_carlo_moving_objects_tracking.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libmoving_objects_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = moving_objects

TARGETS = moving_objects libmoving_objects_interface.a
PUBLIC_LIBRARIES_SO = libmoving_objects_interface.so

ifndef NO_PYTHON
#TARGETS += libmoving_objects.so.1
endif

# If you set graphics support in configure (only for GTK).
ifndef NO_GRAPHICS
FLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += `pkg-config --libs gtk+-2.0`
#Source code file (.c)
TARGETS += 
SOURCES += 
endif

libmoving_objects_interface.so.1:	moving_objects_interface.o moving_objects_types.o

moving_objects:	moving_objects_main.o libmoving_objects_interface.a moving_objects.o monte_carlo_moving_objects_tracking.o

libmoving_objects_interface.a:	moving_objects_interface.o moving_objects.a monte_carlo_moving_objects_tracking.a

# rules

include ../Makefile.rules
