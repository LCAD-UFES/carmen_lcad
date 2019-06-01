include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = SEG MAP UTIL TESTS
MODULE_COMMENT = Tests programs of the libsegmap_util library 

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
		-lxsens_interface -lgps_nmea_interface

# g2o libs
LFLAGS += -L/usr/local/lib -lcholmod -lg2o_core -lg2o_types_slam2d -lg2o_solver_cholmod -lg2o_stuff

LFLAGS += -lvtkCommon -lvtkFiltering -lvtkRendering -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search \
		-lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization  -lboost_system -lboost_thread-mt
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -L$(CARMEN_HOME)/sharedlib/gl_plotter -lprob_models -lrotation_geometry
LFLAGS += -lmapper_interface -lgrid_mapping -lmap_io -lmap_interface
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lpthread -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system -lgomp -lvtkGraphics 
LFLAGS += `pkg-config --libs opencv` -lboost_system

LFLAGS += -lsegmap_initializations 

IFLAGS += -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ -I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`
LFLAGS += -L /usr/local/lib -lpcl_common -lpcl_io -lpcl_visualization -lglobal -lboost_program_options `pkg-config --libs opencv` 

LFLAGS += -lsegmap_datasets -lsegmap_readers -lsegmap_slam \
	-lsegmap_visualization -lsegmap_readers -lsegmap_motion_models -lcarmen_util `pkg-config --libs opencv` -lboost_filesystem -lsegmap_types  

#LFLAGS += -lsegmap_cells -lsegmap_distributions

LFLAGS += -lvtkRendering -lvtkCommon -lvtkFiltering -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search -lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization  -lboost_system -lboost_thread-mt -lrt -lboost_signals -lboost_system

DEPS = $(CARMEN_HOME)/src/segmap/libsegmap/types/libsegmap_types.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/datasets/libsegmap_datasets.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/readers/libsegmap_readers.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/motion_models/libsegmap_motion_models.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/readers/libsemap_initializations.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/slam/libsegmap_slam.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/visualization/libsegmap_visualization.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/readers/libsemap_readers.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/slam/hash_map/distributions/libsegmap_distributions.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/slam/hash_map/cells/libsegmap_cells.a \
	$(CARMEN_HOME)/src/utilities/libcarmen_util/libcarmen_util.a 

SOURCES = test_velodyne_camera_calibration.cpp # test_xsens_vs_gps_angle.cpp test_segmap_dataset.cpp test_command_line.cpp test_changing_camera_position_pcl.cpp #test_segmap_hash_map.cpp
TARGETS = test_velodyne_camera_calibration # test_xsens_vs_gps_angle test_command_line test_changing_camera_position_pcl #test_segmap_hash_map

test_velodyne_camera_calibration: test_velodyne_camera_calibration.o $(DEPS)
test_xsens_vs_gps_angle: test_xsens_vs_gps_angle.o $(DEPS)
test_changing_camera_position_pcl: test_changing_camera_position_pcl.o $(DEPS) 
test_segmap_dataset: test_segmap_dataset.o $(DEPS)
test_command_line: test_command_line.o $(DEPS)
#test_segmap_hash_map: test_segmap_hash_map.o $(DEPS)

include $(CARMEN_HOME)/src/Makefile.rules
