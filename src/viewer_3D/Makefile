include ../Makefile.conf

CFLAGS +=
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += 	-lglobal -ljpeg -lipc -lparam_interface -lGLEW -lGL -lGLU -lglut -lX11 -llaser_interface \
		-lfused_odometry_interface -lgps_xyz_interface -lxsens_interface -lstereo_point_cloud_interface -lglm -lrotation_geometry\
		-lcurl `pkg-config --libs opencv` \
		-lvelodyne_interface -lcarmen_gps_wrapper -lm -ldownload_map -lstereo_velodyne_interface -lstereo_velodyne \
		-lstereo_interface -lmapper_interface -lnavigator_ackerman_interface -lbehavior_selector_interface \
		-llocalize_ackerman_interface -lobstacle_avoider_interface -lmotion_planner_interface -lrddf_interface \
		-lmoving_objects_interface -lgps_nmea_interface -llaser_ldmrs_interface #-llane_analysis_interface
		
LFLAGS += -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system -ljpeg

# Application specific include directories.
IFLAGS += -I/usr/include/eigen3 -I/usr/local/include/pcl-1.7/  -I/usr/include/vtk-5.8/
LFLAGS += -lvtkCommon -lvtkFiltering -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search -lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization  -lboost_system -lboost_thread-mt -lrt -lboost_signals -lboost_system

SUBDIRS += glm

MODULE_NAME = viewer_3D
MODULE_COMMENT = This module implements a viewer_3D filter for fusing IMU and GPS data.

SOURCES = viewer_3D.cpp Window.c GLDraw.c xsens_viewer.c draw_car.c point_cloud_drawer.c variable_velodyne_drawer.cpp velodyne_360_drawer.c interface_drawer.cpp map_drawer.c download_map.c texture_loader.c trajectory_drawer.c velodyne_intensity_drawer.c annotation_drawer.cpp lane_analysis_drawer.cpp
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

viewer_3D:	viewer_3D.o Window.o GLDraw.o draw_car.o point_cloud_drawer.o variable_velodyne_drawer.o velodyne_360_drawer.o interface_drawer.o map_drawer.o texture_loader.o download_map.o trajectory_drawer.o velodyne_intensity_drawer.o annotation_drawer.o lane_analysis_drawer.o

xsens_viewer: xsens_viewer.o Window.o draw_car.o texture_loader.o download_map.o

include ../Makefile.rules



