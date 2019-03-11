include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = SEGMAP_SCRIPTS_PACKAGE_MISC
MODULE_COMMENT = Scripts for segmaps - Miscellaneous

LINK = g++ 

CXXFLAGS += -Wno-deprecated -Wno-deprecated-declarations -Wno-sign-compare

CFLAGS += -std=c11 -fopenmp
CXXFLAGS += -std=c++11 -fopenmp

# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11

PCL_INC = $(wildcard /usr/local/include/pcl-* /usr/include/pcl-*)
VTK_INC = $(wildcard /usr/local/include/vtk-5* /usr/include/vtk-5*)
IFLAGS += -I/usr/include/eigen3 -I $(PCL_INC) -I $(VTK_INC)

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/  
IFLAGS += -I/usr/include/eigen3 
IFLAGS += -I/usr/include/suitesparse -I/usr/include/eigen3 -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -I$(CARMEN_HOME)/sharedlib/gl_plotter 
IFLAGS += `pkg-config --cflags opencv` 

LFLAGS += -lglobal -lipc -llocalize_ackerman_interface -lfused_odometry_interface -lvelodyne_interface -lparam_interface -lGL -lglut -lGLU \
		-lrt -lboost_signals -std=c++11 # -ltf_util    
LFLAGS += -lrobot_ackerman_interface -lobstacle_avoider_interface \
		-lmap_io -lmap_util -llaser_interface -lfused_odometry_interface -lvelodyne_interface \
		-lGLEW -lGL -lGLU -lglut -lX11 -lmap_interface -lbase_ackerman_interface \
		-lstereo_velodyne -lstereo_velodyne_interface -lstereo_interface -llocalize_ackerman_core -llocalize_ackerman_interface \
		-llocalize_ackerman_motion -lmap_server_interface -lgps_xyz_interface -lvisual_odometry_interface -lxsens_mtig_interface \
		-lxsens_interface -lgps_nmea_interface -lboost_filesystem
LFLAGS += -L /usr/local/lib -lcholmod -lg2o_core -lg2o_stuff -lg2o_types_slam2d -lg2o_cli
LFLAGS += -lvtkCommon -lvtkFiltering -lvtkRendering -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search \
		-lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization  -lboost_system -lboost_thread-mt
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -L$(CARMEN_HOME)/sharedlib/gl_plotter -lprob_models -lrotation_geometry
LFLAGS += `pkg-config --libs opencv`	  
LFLAGS += -lmapper_interface -lgrid_mapping -lmap_io -lmap_interface
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lpthread -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system -lgomp 
LFLAGS += -lsegmap
LFLAGS += `pkg-config --libs opencv` -lvtkCommon -lvtkFiltering -lvtkRendering -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search \
		-lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization  -lboost_system -lboost_thread-mt -lglobal -lsegmap

SOURCES = generate_dataset_velodyne_segmentation.cpp generate_data_for_cam_lidar_calibration.cpp cam_lidar_calibration_tool.cpp
TARGETS = generate_dataset_velodyne_segmentation generate_data_for_cam_lidar_calibration cam_lidar_calibration_tool

generate_dataset_velodyne_segmentation: generate_dataset_velodyne_segmentation.o 
generate_data_for_cam_lidar_calibration: generate_data_for_cam_lidar_calibration.o
cam_lidar_calibration_tool: cam_lidar_calibration_tool.o

include $(CARMEN_HOME)/src/Makefile.rules
