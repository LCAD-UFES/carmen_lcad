include ../Makefile.conf

# Migração Ubuntu 26.04: o módulo linkava contra o OpenCV mas nunca pedia os -I dele.
# No Ubuntu 20.04 os headers ficavam em /usr/include/opencv2 (path padrão); o pacote do
# 26.04 põe tudo em /usr/include/opencv4, que exige -I explícito.
IFLAGS += `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv`

MODULE_NAME = POST SLAM
MODULE_COMMENT = Slam Module using Velodyne, Unscented Kalman Filter and Posts as Landmarks

CXXFLAGS += -std=c++14
LINK = g++

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I$(CARMEN_HOME)/sharedlib/prob_models 
LFLAGS += -lglobal -lipc -lGL -lglut -lGLU -lvelodyne_interface -lparam_interface `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv` \
	 -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread -lrt \
	 -lrotation_geometry -llocalize_ackerman_interface -lfused_odometry_interface -lpthread
	 
SOURCES = post_mapping_main.cpp post_localize_main.cpp post_localize.cpp landmark.cpp landmark_map.cpp

PUBLIC_INCLUDES =  
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = post_mapping post_localize

TARGETS = post_mapping post_localize

post_mapping: post_mapping_main.o landmark.o landmark_map.o
post_localize: post_localize_main.o landmark.o landmark_map.o post_localize.o

include ../Makefile.rules
