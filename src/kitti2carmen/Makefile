include ../Makefile.conf

# Module name and description
MODULE_NAME = KITTI2CARMEN
MODULE_COMMENT = KITTI TO CARMEN

LINK = g++

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I$(CARMEN_HOME)/sharedlib/prob_models  -fopenmp 

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += `pkg-config --libs opencv`
# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lipc -lglobal -lvelodyne_interface -lparam_interface -lwritelog -lkinect_interface -lbumblebee_basic_interface -lxsens_interface -lgps_nmea_interface 
LFLAGS += -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system -pthread
CFLAGS += -Wall

# Source code files (.c, .cpp) 
SOURCES = kitti2carmen_main.cpp read_kitti.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = 

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = 

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = kitti2carmen_main

TARGETS = kitti2carmen_main

kitti2carmen_main: kitti2carmen_main.o read_kitti.o

include ../Makefile.rules

