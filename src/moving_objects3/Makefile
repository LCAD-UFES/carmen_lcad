include ../Makefile.conf

MODULE_NAME = Moving Objects
MODULE_COMMENT = Moving objects using 1-D approach

LINK = g++

CFLAGS += -Wall -std=c99
CXXFLAGS += -Wall -std=c++98

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I$(CARMEN_HOME)/sharedlib/prob_models  -fopenmp 
LFLAGS += -lglobal -lipc -llaser_interface -lGL -lglut -lGLU -lvelodyne_interface -lparam_interface `pkg-config --libs opencv` \
	 -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system \
	 -L$(CARMEN_HOME)/sharedlib/prob_models -lprob_models -lmap_interface -lrotation_geometry -lfused_odometry_interface -lbase_ackerman_interface

SOURCES = moving_objects3_interface.c tf_util.cpp \
	moving_objects3_main.cpp polar_point.cpp moving_objects3_viewer.cpp moving_objects3_particle_filter.cpp moving_objects3_utils.cpp 

PUBLIC_INCLUDES = moving_objects3_interface.h moving_objects3_particle_filter.h moving_objects3_messages.h polar_point.h tf_util.h
PUBLIC_LIBRARIES = libtf_util.a
PUBLIC_BINARIES = moving_objects3 moving_objects3_viewer

TARGETS = libtf_util.a moving_objects3 moving_objects3_viewer

libtf_util.a: tf_util.o
moving_objects3: tf_util.o moving_objects3_main.o moving_objects3_interface.o moving_objects3_utils.o moving_objects3_particle_filter.o polar_point.o 
moving_objects3_viewer: moving_objects3_viewer.o polar_point.o moving_objects3_interface.o moving_objects3_utils.o

include ../Makefile.rules
