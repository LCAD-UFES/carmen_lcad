include ../Makefile.conf

CFLAGS += -std=c99 -fopenmp -O4

LINK = g++

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -I$(CARMEN_HOME)/sharedlib/gl_plotter 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -L$(CARMEN_HOME)/sharedlib/gl_plotter

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src 


LFLAGS += -lvelodyne_interface -lrotation_geometry -lglobal -lparam_interface -lmap_io -lmap_util -llaser_interface \
	  -lrobot_ackerman_interface -lobstacle_avoider_interface -lipc -lmap_server_interface \
	  -lfused_odometry_interface -lprob_models -lgl_plotter \
	  -lGLEW -lGL -lGLU -lglut -lX11 -lmap_interface -lbase_ackerman_interface \
	  -lstereo_velodyne -lstereo_velodyne_interface -lstereo_interface

LFLAGS += -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system

IFLAGS += `pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv`


PCL_INC = $(wildcard /usr/local/include/pcl-*)
VTK_INC = $(wildcard /usr/local/include/vtk-5* /usr/include/vtk-5*)
IFLAGS += -I/usr/include/eigen3 -I $(PCL_INC) -I $(VTK_INC)

LFLAGS += -lvtkCommon -lvtkFiltering -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search -lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization  -lboost_system -lboost_thread-mt -lrt -lboost_signals -lboost_system


MODULE_NAME = SLAM ICP
MODULE_COMMENT = ICP SLAM Module

SOURCES = slam_icp.cpp slam_icp_velodyne.cpp transformation.cpp
	  
PUBLIC_INCLUDES = 
		  
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = slam_icp transformation

MAN_PAGES =

TARGETS = slam_icp libprob_models.a transformation

PUBLIC_LIBRARIES_SO = 
ifndef NO_PYTHON
TARGETS += 
endif

# rules

libprob_models.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/prob_models/

slam_icp:	slam_icp.o slam_icp_velodyne.o

transformation: transformation.o

clean: clean_prob_models

clean_prob_models:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/prob_models/

include ../Makefile.rules



