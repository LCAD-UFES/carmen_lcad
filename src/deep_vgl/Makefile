include ../Makefile.conf

MODULE_NAME = Localize Neural 2.0
MODULE_COMMENT = Classifies the images of the environment and delivers the location.

LINK = g++ -g
CXXFLAGS = -std=c++11 -g

IFLAGS += -I$(CARMEN_HOME)/sharedlib/darknet4/src -I$(CARMEN_HOME)/sharedlib/darknet4/include 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet4/


# flags for GICP

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/  
IFLAGS += -I/usr/include/eigen3 
IFLAGS += -I/usr/include/suitesparse -I/usr/include/eigen3 -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/

PCL_INC = $(wildcard /usr/local/include/pcl-*)
VTK_INC = $(wildcard /usr/include/vtk-6*)
IFLAGS += -I/usr/include/eigen3 -I $(PCL_INC) -I $(VTK_INC)

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -I$(CARMEN_HOME)/sharedlib/gl_plotter 
IFLAGS += `pkg-config --cflags opencv` `pkg-config --cflags bullet`

LFLAGS += -lglobal -lipc -pthread -llocalize_ackerman_interface -lfused_odometry_interface -lvelodyne_interface -lparam_interface -lGL -lglut -lGLU \
	 	-L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lboost_system \
		-lrt -lboost_signals 
LFLAGS += -lrobot_ackerman_interface -lobstacle_avoider_interface \
		-lmap_io -lmap_util -llaser_interface -lfused_odometry_interface -lvelodyne_interface \
		-lGLEW -lGL -lGLU -lglut -lX11 -lmap_interface -lbase_ackerman_interface \
		-lstereo_velodyne -lstereo_velodyne_interface -lstereo_interface -llocalize_ackerman_core -llocalize_ackerman_interface \
		-llocalize_ackerman_motion -lmap_server_interface -lgps_xyz_interface -lvisual_odometry_interface -lxsens_mtig_interface \
		-lxsens_interface -lgps_nmea_interface
LFLAGS += -L /usr/local/lib -lcholmod -lg2o_core -lg2o_stuff -lg2o_types_slam2d -lg2o_cli

ifeq ($(UBUNTU_VERSION),"16.04")
	LFLAGS += -lvtkCommon -lvtkFiltering -lvtkRenderingCore -lvtkCommonDataModel -lvtkCommonMath -lvtkCommonCore
else
	LFLAGS += -lvtkRenderingCore-6.3 -lvtkCommonDataModel-6.3 -lvtkCommonMath-6.3 -lvtkCommonCore-6.3 
endif

LFLAGS += -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search \
		-lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization  -lboost_system -lboost_thread-mt -lrt -lboost_signals -lboost_system


LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -L$(CARMEN_HOME)/sharedlib/gl_plotter -lprob_models -lrotation_geometry
LFLAGS += `pkg-config --libs opencv`	  
LFLAGS += -lmapper_interface -lgrid_mapping -lmap_io -lmap_interface

# fim da flags GICP



CFLAGS += `pkg-config --cflags opencv` -I/usr/include/gflags

LFLAGS += -lparam_interface -lipc -lglobal -lbumblebee_basic_interface -lgps_xyz_interface -lcamera_drivers -lvelodyne_interface
LFLAGS += -fPIC -lgflags -ldarknet4 -lstdc++ `pkg-config --libs opencv` 

SOURCES = deep_vgl-test.cpp deep_vgl-eval.cpp deep_vgl_main.cpp deep_vgl_icp_test.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = deep_vgl-test deep_vgl-eval deep_vgl deep_vgl_icp_test

TARGETS = darknet deep_vgl-test deep_vgl-eval deep_vgl deep_vgl_icp_test

deep_vgl-test: deep_vgl-test.o

deep_vgl-eval: deep_vgl-eval.o

deep_vgl: deep_vgl_main.o

deep_vgl_icp_test: deep_vgl_icp_test.o

darknet:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/darknet4  > /dev/null 2>&1

clean: clean_localize_neural2

clean_darknet:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/darknet4

clean_localize_neural2:
	rm -f $(TARGETS)
	
include ../Makefile.rules




