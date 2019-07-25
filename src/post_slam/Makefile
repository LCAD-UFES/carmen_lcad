include ../Makefile.conf

MODULE_NAME = POST SLAM
MODULE_COMMENT = Slam Module using Velodyne, Unscented Kalman Filter and Posts as Landmarks

CXXFLAGS += -std=c++11
LINK = g++

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I$(CARMEN_HOME)/sharedlib/prob_models 
LFLAGS += -lglobal -lipc -lGL -lglut -lGLU -lvelodyne_interface -lparam_interface `pkg-config --libs opencv` \
	 -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system \
	 -lrotation_geometry -llocalize_ackerman_interface -lfused_odometry_interface -lpthread
	 
SOURCES = post_mapping_main.cpp post_localize_main.cpp post_localize.cpp landmark.cpp landmark_map.cpp

PUBLIC_INCLUDES =  
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = post_mapping post_localize

TARGETS = post_mapping post_localize

post_mapping: post_mapping_main.o landmark.o landmark_map.o
post_localize: post_localize_main.o landmark.o landmark_map.o post_localize.o

include ../Makefile.rules
