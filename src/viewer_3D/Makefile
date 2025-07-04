include ../Makefile.conf

#CC = g++
#LINK = g++

CFLAGS += -fopenmp
CXXFLAGS += -std=c++11 -Wno-write-strings -Wno-deprecated-declarations

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libsickldmrs2/include
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ 

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libsickldmrs2 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L/usr/local/carmen_boost/lib
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models

LFLAGS += -lglobal -ljpeg -lipc -lparam_interface -lGLEW -lGL -lGLU -lglut -lX11 -llaser_interface \
		-lfused_odometry_interface -lgps_xyz_interface -lxsens_interface -lstereo_point_cloud_interface -lglm -lrotation_geometry \
		-lcurl  \
		-lvelodyne_interface -lcarmen_gps_wrapper -lm -ldownload_map -lstereo_velodyne_interface -lstereo_velodyne \
		-lstereo_interface -lnavigator_ackerman_interface -lbehavior_selector_interface \
		-llocalize_ackerman_interface -lobstacle_avoider_interface -lmotion_planner_interface -lrddf_interface \
		-lmoving_objects_interface -lgps_nmea_interface -llaser_ldmrs_utils -llaser_ldmrs_interface \
		-lmapper_interface -lmap_server_interface -lprob_models -lgrid_mapping -lmap_interface -lmap_io -lfrenet_path_planner_interface \
		-lviewer_3D_interface -lbase_ackerman_interface -loffroad_planner_interface -ltask_manager_interface -lcargo_interface #-llane_analysis_interface
		
LFLAGS += -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system -ljpeg

IFLAGS += `pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv`

# Application specific include directories.
PCL_INC = $(wildcard /usr/local/include/pcl-*)
#VTK_INC = $(wildcard /usr/local/include/vtk-5* /usr/include/vtk-5* /usr/include/vtk-6*)

IFLAGS += -I/usr/include/eigen3 -I $(PCL_INC) # -I $(VTK_INC)

LFLAGS += -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search \
		  -lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization  -lboost_system \
		  -lboost_thread-mt -lrt -lboost_signals -lboost_system -lcollision_detection
LFLAGS += -luser_preferences

SUBDIRS += glm

MODULE_NAME = viewer_3D
MODULE_COMMENT = This module implements a viewer_3D filter for fusing IMU and GPS data.

SOURCES = viewer_3D.cpp Window.cpp GLDraw.cpp xsens_viewer.cpp draw_car.cpp point_cloud_drawer.cpp variable_velodyne_drawer.cpp \
	velodyne_360_drawer.cpp interface_drawer.cpp map_drawer.cpp download_map.cpp texture_loader.cpp trajectory_drawer.cpp \
	velodyne_intensity_drawer.cpp annotation_drawer.cpp symotha_drawer.cpp cargo_drawer.cpp viewer_3D_interface.c 3D_map_view.cpp xyz_pointcloud_lidar_interface.cpp#lane_analysis_drawer.cpp
	
PUBLIC_INCLUDES = velodyne_360_drawer.h viewer_3D_messages.h viewer_3D_interface.h xyz_pointcloud_lidar_interface.h xyz_pointcloud_lidar_messages.h
PUBLIC_LIBRARIES = libvelodyne_360_drawer.a libviewer_3D_interface.a libxyz_pointcloud_lidar_interface.a
PUBLIC_BINARIES = viewer_3D xsens_viewer
PUBLIC_OTHERS = Touareg2.mtl Touareg2.obj

TARGETS = viewer_3D xsens_viewer libvelodyne_360_drawer.a libviewer_3D_interface.a 3D_map_view libxyz_pointcloud_lidar_interface.a

libxyz_pointcloud_lidar_interface.a: xyz_pointcloud_lidar_interface.o

libviewer_3D_interface.a: viewer_3D_interface.o

libvelodyne_360_drawer.a: velodyne_360_drawer.o

viewer_3D:	viewer_3D.o Window.o GLDraw.o draw_car.o point_cloud_drawer.o variable_velodyne_drawer.o velodyne_360_drawer.o \
	interface_drawer.o map_drawer.o texture_loader.o download_map.o trajectory_drawer.o velodyne_intensity_drawer.o \
	annotation_drawer.o symotha_drawer.o cargo_drawer.o libviewer_3D_interface.a libxyz_pointcloud_lidar_interface.a#lane_analysis_drawer.o

xsens_viewer: xsens_viewer.o Window.o draw_car.o texture_loader.o download_map.o

3D_map_view: 3D_map_view.o libviewer_3D_interface.a

include ../Makefile.rules
