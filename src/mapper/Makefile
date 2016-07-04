include ../Makefile.conf

# Module name and description
MODULE_NAME = MAPPER_PACKAGE
MODULE_COMMENT = Mapper Maps using Velodyne!

CFLAGS += -Wall -std=c99 -msse3 -O4 -fopenmp -funroll-all-loops
#CFLAGS += -g -pg
LINK = g++

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -fopenmp
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -fopenmp

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src


# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lgrid_mapping -lmap_io -lmap_util -lmap_interface -lipc -lprob_models -lm -lz -lglobal 		\
	-lparam_interface -llaser_interface -lipc -lgrid_mapping_interface -lobstacle_avoider_interface		\
	 -llocalize_ackerman_interface -lmap_interface -lgrid_mapping -lipc		\
	-lultrasonic_filter_interface -lstereo_mapping_interface -lrotation_geometry -lvelodyne_interface	\
	-lfused_odometry_interface  -lstereo_velodyne -lstereo_velodyne_interface -lstereo_interface		\
	`pkg-config --libs opencv` -lgeometry -lmap_server_interface -lsimulator_ackerman_interface -lrddf_interface \
	-lparking_assistant_interface  -llocalize_ackerman_core 

LFLAGS += -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system

# Application specific include directories.
IFLAGS += -I/usr/include/eigen3 -I/usr/local/include/pcl-1.7/  -I/usr/include/vtk-5.8/
LFLAGS += -lvtkCommon -lvtkFiltering -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search -lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization  -lboost_system -lboost_thread-mt -lrt -lboost_signals -lboost_system

	
# Source code files (.c, .cpp) 
SOURCES = mapper.cpp mapper_main.cpp message_interpolation.cpp convert_maps_from_float_to_double.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = 

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = 

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = mapper
TARGETS = convert_maps_from_float_to_double libprob_models.a mapper

mapper: mapper.o mapper_main.o 

convert_maps_from_float_to_double: convert_maps_from_float_to_double.o

# rules

libprob_models.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/prob_models/

clean: clean_prob_models

clean_prob_models:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/prob_models/

include ../Makefile.rules
