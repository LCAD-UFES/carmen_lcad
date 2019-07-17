include ../Makefile.conf

MODULE_NAME = POLAR SLAM
MODULE_COMMENT = Polar SLAM

CXXFLAGS += -std=c++11
LINK = g++

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I$(CARMEN_HOME)/sharedlib/prob_models  -fopenmp 
LFLAGS += -lglobal -lipc -llaser_interface -lGL -lglut -lGLU -lvelodyne_interface -lparam_interface `pkg-config --libs opencv` \
	 -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system \
	 -L$(CARMEN_HOME)/sharedlib/prob_models -lprob_models -lmap_interface -lrotation_geometry -lfused_odometry_interface -lbase_ackerman_interface    
	 
CFLAGS += -pg

SOURCES = fast_polar_slam_interface.c tf_util.cpp fast_polar_particle_filter.cpp \
	fast_polar_slam_main.cpp polar_point.cpp fast_polar_slam_viewer.cpp 

PUBLIC_INCLUDES = fast_polar_slam_interface.h fast_polar_slam_messages.h fast_polar_particle_filter.h polar_point.h tf_util.h
PUBLIC_LIBRARIES = libtf_util.a
PUBLIC_BINARIES = fast_polar_slam fast_polar_slam_viewer

TARGETS = libtf_util.a fast_polar_slam fast_polar_slam_viewer

libtf_util.a: tf_util.o
fast_polar_slam: tf_util.o fast_polar_slam_main.o fast_polar_slam_interface.o polar_point.o fast_polar_particle_filter.o 
fast_polar_slam_viewer: fast_polar_slam_viewer.o polar_point.o fast_polar_slam_interface.o 

include ../Makefile.rules
