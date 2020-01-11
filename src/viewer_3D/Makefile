include ../Makefile.conf

LINK = g++

CFLAGS += -fopenmp
CXXFLAGS += -std=c++11 -Wno-write-strings

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libsickldmrs2/include
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libsickldmrs2 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models

LFLAGS += -lglobal -ljpeg -lipc -lparam_interface -lGLEW -lGL -lGLU -lglut -lX11 -llaser_interface \
		-lfused_odometry_interface -lgps_xyz_interface -lxsens_interface -lstereo_point_cloud_interface -lglm -lrotation_geometry\
		-lcurl  \
		-lvelodyne_interface -lcarmen_gps_wrapper -lm -ldownload_map -lstereo_velodyne_interface -lstereo_velodyne \
		-lstereo_interface -lnavigator_ackerman_interface -lbehavior_selector_interface \
		-llocalize_ackerman_interface -lobstacle_avoider_interface -lmotion_planner_interface -lrddf_interface \
		-lmoving_objects_interface -lgps_nmea_interface -llaser_ldmrs_utils -llaser_ldmrs_interface -llocalize_neural_interface \
		-lmapper_interface -lmap_server_interface -lprob_models -lgrid_mapping -lmap_interface -lmap_io #-llane_analysis_interface
		
LFLAGS += -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system -ljpeg

IFLAGS += `pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv`

# Application specific include directories.
PCL_INC = $(wildcard /usr/local/include/pcl-* /usr/include/pcl-*)
VTK_INC = $(wildcard /usr/local/include/vtk-5* /usr/include/vtk-5*)

IFLAGS += -I/usr/include/eigen3 -I $(PCL_INC) -I $(VTK_INC)

LFLAGS += -lvtkCommon -lvtkFiltering -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search \
		  -lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization  -lboost_system \
		  -lboost_thread-mt -lrt -lboost_signals -lboost_system -lcollision_detection

SUBDIRS += glm

MODULE_NAME = viewer_3D
MODULE_COMMENT = This module implements a viewer_3D filter for fusing IMU and GPS data.

SOURCES = viewer_3D.cpp Window.cpp GLDraw.cpp xsens_viewer.cpp draw_car.cpp point_cloud_drawer.cpp variable_velodyne_drawer.cpp \
	velodyne_360_drawer.cpp interface_drawer.cpp map_drawer.cpp download_map.cpp texture_loader.cpp trajectory_drawer.cpp \
	velodyne_intensity_drawer.cpp annotation_drawer.cpp symotha_drawer.cpp #lane_analysis_drawer.cpp
	
PUBLIC_INCLUDES = velodyne_360_drawer.h

PUBLIC_LIBRARIES = libvelodyne_360_drawer.a
PUBLIC_BINARIES = viewer_3D xsens_viewer
MAN_PAGES =

TARGETS = viewer_3D xsens_viewer libvelodyne_360_drawer.a

PUBLIC_LIBRARIES_SO =

ifndef NO_PYTHON
TARGETS += 
endif

# rules
libvelodyne_360_drawer.a: velodyne_360_drawer.o

viewer_3D:	viewer_3D.o Window.o GLDraw.o draw_car.o point_cloud_drawer.o variable_velodyne_drawer.o velodyne_360_drawer.o \
	interface_drawer.o map_drawer.o texture_loader.o download_map.o trajectory_drawer.o velodyne_intensity_drawer.o \
	annotation_drawer.o symotha_drawer.o #lane_analysis_drawer.o

xsens_viewer: xsens_viewer.o Window.o draw_car.o texture_loader.o download_map.o

include ../Makefile.rules



