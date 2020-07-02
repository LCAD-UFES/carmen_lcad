include ../Makefile.conf

# Module name and description
MODULE_NAME = clean_map_PACKAGE
MODULE_COMMENT = clean map Velodyne!

CFLAGS += -Wall -std=c99  -mfpmath=sse -msse2 -msse4.2 -ffast-math -fassociative-math -O -fopenmp -funroll-all-loops
#CFLAGS += -g -pg
LINK = g++

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -fopenmp
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -fopenmp

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src


# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lgrid_mapping -lmapper -lmap_io -lmap_util -lmap_interface -lipc -lprob_models -lm -lz -lglobal 		\
	-lparam_interface -llaser_interface -lipc -lmapper_interface -lobstacle_avoider_interface		\
	 -llocalize_ackerman_interface -lmap_interface -lmapper -lipc		\
	-lultrasonic_filter_interface -lstereo_mapping_interface -lrotation_geometry -lvelodyne_interface	\
	-lfused_odometry_interface  -lstereo_velodyne -lstereo_velodyne_interface -lstereo_interface		\
	`pkg-config --libs opencv` -lgeometry -lmap_server_interface -lsimulator_ackerman_interface -lrddf_interface \
	-lparking_assistant_interface  -llocalize_ackerman_core 

LFLAGS += -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system

# Application specific include directories.
PCL_INC = $(wildcard /usr/local/include/pcl-*)
VTK_INC = $(wildcard /usr/local/include/vtk-5* /usr/include/vtk-5*)
IFLAGS += -I/usr/include/eigen3 -I $(PCL_INC) -I $(VTK_INC)

LFLAGS += -lvtkCommon -lvtkFiltering -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search -lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization  -lboost_system -lboost_thread-mt -lrt -lboost_signals -lboost_system

	
# Source code files (.c, .cpp) 
SOURCES = clean_map.cpp clean_map_main.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = clean_map.h

# Public libraries, linked to 'carmen/lib'.
#PUBLIC_LIBRARIES =

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = clean_map
TARGETS = libprob_models.a clean_map 

clean_map: clean_map.o clean_map_main.o

# rules

libprob_models.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/prob_models/

#clean: clean_prob_models
#
#clean_prob_models:
#	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/prob_models/

include ../Makefile.rules

