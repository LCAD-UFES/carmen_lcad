include ../Makefile.conf

MODULE_NAME = GRAPHSLAM_PACKAGE
MODULE_COMMENT = teste_grab_data

LINK = g++ 

CXXFLAGS += -Wno-deprecated -Wno-deprecated-declarations

CFLAGS += -std=c11
CXXFLAGS += -std=c++11

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
LFLAGS += -L /usr/local/lib -lcholmod -lg2o_core -lg2o_stuff -lg2o_types_slam2d -lg2o_cli
LFLAGS += -lvtkCommon -lvtkFiltering -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search \
		-lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization  -lboost_system -lboost_thread-mt
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -L$(CARMEN_HOME)/sharedlib/gl_plotter -lprob_models -lrotation_geometry
LFLAGS += `pkg-config --libs opencv`	  
LFLAGS += -lmapper_interface -lgrid_mapping -lmap_io -lmap_interface
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lpthread -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system -lgomp -lcarmen_util

SOURCES = grab_data.cpp velodyne_util.cpp graphslam.cpp run_icp_for_loop_closure.cpp graphslam_publish_main.cpp graphslam_params.cpp multimap_optimization.cpp multimap_matching.cpp gen_data_to_plot.cpp pointcloud_viewer.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = grab_data run_icp_for_loop_closure graphslam graphslam_publish multimap_matching multimap_optimization pointcloud_viewer
TARGETS = grab_data graphslam run_icp_for_loop_closure graphslam_publish multimap_matching multimap_optimization gen_data_to_plot pointcloud_viewer #run_icp   

pointcloud_viewer: pointcloud_viewer.o
run_icp: run_icp.o
graphslam: graphslam.o
graphslam_publish: graphslam_publish_main.o
run_icp_for_loop_closure: run_icp_for_loop_closure.o
grab_data: grab_data.o velodyne_util.o graphslam_params.o
multimap_matching: multimap_matching.o
multimap_optimization: multimap_optimization.o
gen_data_to_plot: gen_data_to_plot.o

include ../Makefile.rules
