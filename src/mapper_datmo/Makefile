include ../Makefile.conf

# Module name and description
MODULE_NAME = MAPPER_DATMO
MODULE_COMMENT = Creates maps using velodyne and camera image segmentation for detection of moving objects

#CFLAGS += # -Wall -std=c99  -O3 -mfpmath=sse -msse2 -msse4.2 -ffast-math -fassociative-math -O4 -fopenmp -funroll-all-loops -Wno-unused-but-set-variable
#CFLAGS += -Wall -std=c99 -O4 -mfpmath=sse -msse2 -msse4.2
#CXXFLAGS += -Wall -O4 -mfpmath=sse -msse2 -msse4.2
CFLAGS += -std=c11
CXXFLAGS += -std=c++11
#CFLAGS += -g -pg
LINK = g++

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -fopenmp
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -fopenmp

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src

IFLAGS += `pkg-config --cflags opencv`

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libsickldmrs2/include
# Required default libraries to comunicate with Carmen Core.
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libsickldmrs2 

# Deeplab
IFLAGS += -I/usr/include/python2.7
LFLAGS += -ldeeplab -lpython2.7
LFLAGS += -lsqueeze_seg -lpython2.7

#OPENCV
IFLAGS += -DUSE_OPENCV
LFLAGS += `pkg-config opencv --libs` `pkg-config --cflags opencv` `pkg-config --libs opencv`

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lgrid_mapping -lmapper -lmap_io -lmap_util -lmap_interface -lipc -lprob_models -lm -lz -lglobal 		\
	-lparam_interface -llaser_interface -lipc -lmapper_interface -lobstacle_avoider_interface		\
	-llocalize_ackerman_interface -lmap_interface -lmapper -lipc		\
	-lultrasonic_filter_interface -lstereo_mapping_interface -lrotation_geometry -lvelodyne_interface	\
	-lfused_odometry_interface  -lstereo_velodyne -lstereo_velodyne_interface -lstereo_interface		\
	-lbumblebee_basic_interface -lvelodyne_camera_calibration		\
	`pkg-config --libs opencv` -lgeometry -lmap_server_interface -lsimulator_ackerman_interface -lrddf_interface \
	-lparking_assistant_interface  -llocalize_ackerman_core -llaser_ldmrs_interface -llaser_ldmrs_utils -lmoving_objects_interface

LFLAGS += -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system

# Application specific include directories.
PCL_INC = $(wildcard /usr/local/include/pcl-* /usr/include/pcl-*)
VTK_INC = $(wildcard /usr/local/include/vtk-5* /usr/include/vtk-5*)
IFLAGS += -I/usr/include/eigen3 -I $(PCL_INC) -I $(VTK_INC)

LFLAGS += -lvtkCommon -lvtkFiltering -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search -lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization  -lboost_system -lboost_thread-mt -lrt -lboost_signals -lboost_system
LFLAGS += -lrddf_util -L/usr/lib64/libkml -lkmlbase -lkmldom -lkmlengine -lcarmen_gps_wrapper
	
# Source code files (.c, .cpp) 
SOURCES = mapper.cpp mapper_main.cpp message_interpolation.cpp convert_maps_from_float_to_double.cpp mapper_interface.c compute_velodyne_calibration_table.cpp compute_maps_difference.cpp compute_maps_changes.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = mapper_messages.h mapper_interface.h mapper.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libmapper_interface.a libmapper.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = mapper_datmo compute_velodyne_calibration_table
TARGETS = convert_maps_from_float_to_double libprob_models.a mapper_datmo libmapper_interface.a libmapper.a compute_velodyne_calibration_table libdeeplab.a compute_maps_difference compute_maps_changes libsqueeze_seg.a

# If you set graphics support in configure.
ifndef NO_GRAPHICS
#Carmen Global Graphic Library and Gtk+2 Graphics Libraries
LFLAGS += -lglobal_graphics
IFLAGS += `pkg-config --cflags gtk+-2.0 gmodule-export-2.0`
IFLAGS += `pkg-config --cflags gtkglext-1.0`
LFLAGS += `pkg-config --libs gtk+-2.0 gmodule-export-2.0`
LFLAGS += `pkg-config --libs gtkglext-1.0`
endif

libmapper_interface.a:	mapper_interface.o

libmapper.a: mapper.o

libmapper_interface.so.1:	libmapper_interface.o

mapper_datmo: mapper.o mapper_main.o libmapper.a

convert_maps_from_float_to_double: convert_maps_from_float_to_double.o

compute_velodyne_calibration_table: compute_velodyne_calibration_table.o

compute_maps_difference: compute_maps_difference.o

compute_maps_changes: compute_maps_changes.o

# rules

libprob_models.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/prob_models/

libdeeplab.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/Deeplab/

libsqueeze_seg.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libsqueeze_seg_v2/src/


clean: clean_prob_models

clean_prob_models:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/prob_models/

include ../Makefile.rules
