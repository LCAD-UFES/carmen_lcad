include ../Makefile.conf

# Module name and description
MODULE_NAME = Virtual Depth Estimation
MODULE_COMMENT = Creates maps using camera image detection of moving objects

#CFLAGS += # -Wall -std=c99  -O3 -mfpmath=sse -msse2 -msse4.2 -ffast-math -fassociative-math -O4 -fopenmp -funroll-all-loops -Wno-unused-but-set-variable
#CFLAGS += -Wall -std=c99 -O4 -mfpmath=sse -msse2 -msse4.2
#CXXFLAGS += -Wall -O4 -mfpmath=sse -msse2 -msse4.2
CFLAGS += -std=c11
CXXFLAGS += -std=c++11
#CFLAGS += -g -pg
LINK = g++

# AdaBins
# IFLAGS += -I/usr/include/python2.7
# LFLAGS += `python2.7-config --libs` -ladabins

# DPT
IFLAGS += -I/usr/include/python3.7 -I/usr/include/python3.6m
LFLAGS += `python3.7-config --libs` -ldpt


# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -fopenmp
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -fopenmp

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/include/bullet/ -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src 

#YOLO
LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet4/lib
LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet4
IFLAGS += -I$(CARMEN_HOME)/sharedlib/darknet4

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libsickldmrs2/include
# Required default libraries to comunicate with Carmen Core.
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libsickldmrs2 


#OPENCV
IFLAGS += -DUSE_OPENCV
LFLAGS += `pkg-config opencv --libs` `pkg-config --cflags opencv` `pkg-config --libs opencv`

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lgrid_mapping -lmapper -lmap_io -lmap_util -lmap_interface -lipc -lprob_models -lm -lz -lglobal 		\
	-lparam_interface -llaser_interface -lipc -lmapper_interface -lobstacle_avoider_interface		\
	-llocalize_ackerman_interface -lmap_interface -lmapper -lipc		\
	-lultrasonic_filter_interface -lstereo_mapping_interface -lrotation_geometry -lvelodyne_interface	\
	-lfused_odometry_interface  -lstereo_velodyne -lstereo_velodyne_interface -lstereo_interface		\
	-lbumblebee_basic_interface -lvelodyne_camera_calibration -ldarknet4		\
	`pkg-config --libs opencv` -lgeometry -lmap_server_interface -lsimulator_ackerman_interface -lrddf_interface \
	-lparking_assistant_interface  -llocalize_ackerman_core -llaser_ldmrs_interface -llaser_ldmrs_utils -lmoving_objects_interface \
	-lmapper -lmapper_interface
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
	-lboost_thread-mt -lrt -lboost_signals -lboost_system `python3.6m-config --ldflags --libs`

# LFLAGS += -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system

# Application specific include directories.
PCL_INC = $(wildcard /usr/local/include/pcl-*)
VTK_INC = $(wildcard /usr/local/include/vtk-5* /usr/include/vtk-5*)
IFLAGS += -I/usr/include/eigen3 -I $(PCL_INC) -I $(VTK_INC)

ifeq ($(UBUNTU_VERSION),"16.04")
	LFLAGS += -lvtkCommon -lvtkFiltering -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search -lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization  -lboost_system -lboost_thread-mt -lrt -lboost_signals -lboost_system
else
	LFLAGS +=  -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search -lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization  -lboost_system -lboost_thread-mt -lrt -lboost_signals -lboost_system
endif
LFLAGS +=  -lrddf_util -lcollision_detection -L/usr/lib64/libkml -lkmlbase -lkmldom -lkmlengine -lcarmen_gps_wrapper

# Source code files (.c, .cpp) 
SOURCES = dbscan.cpp movable_object.cpp virtual_depth.cpp virtual_dpt.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = 

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = 

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = virtual_depth virtual_dpt
TARGETS = darknet libtf moving_objects libdpt.a libprob_models.a dbscan movable_object virtual_depth virtual_dpt

# # If you set graphics support in configure.
ifndef NO_GRAPHICS
# #Carmen Global Graphic Library and Gtk+2 Graphics Libraries
LFLAGS += -lglobal_graphics
IFLAGS += `pkg-config --cflags gtk+-3.0`
LFLAGS += `pkg-config --libs gtk+-3.0`
# IFLAGS += `pkg-config --cflags gtk+-2.0 gmodule-export-2.0`
# IFLAGS += `pkg-config --cflags gtkglext-1.0`
# LFLAGS += `pkg-config --libs gtk+-2.0 gmodule-export-2.0`
# LFLAGS += `pkg-config --libs gtkglext-1.0`
endif

dbscan: dbscan.o

movable_object: movable_object.o

virtual_depth: virtual_depth.o $(CARMEN_HOME)/src/virtual_depth_estimator/DPT/libdpt.a dbscan.o movable_object.o

virtual_dpt: virtual_dpt.o $(CARMEN_HOME)/src/virtual_depth_estimator/DPT/libdpt.a dbscan.o movable_object.o

# rules
libtf:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libtf/src

libdpt.a: 
	$(MAKE) -C $(CARMEN_HOME)/src/virtual_depth_estimator/DPT

moving_objects: 
	$(MAKE) -C $(CARMEN_HOME)/src/moving_objects/

darknet:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/darknet4

include ../Makefile.rules
