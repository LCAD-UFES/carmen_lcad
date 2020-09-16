include ../Makefile.conf

# Module name and description
MODULE_NAME = OBSTACLE_DISTANCE_MAPPER
MODULE_COMMENT = Obstacle Distance Mapper Module

#To debug
#CFLAGS += -Wall -std=c++11 -O0
#CXXFLAGS += -Wall -std=c++11 -O0

#To release
CFLAGS += -Wall
CXXFLAGS += -Wall -std=c++14

LINK = g++

# Application specific include directories. 
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -fopenmp
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -fopenmp

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lipc -lglobal -lobstacle_distance_mapper_interface \
		   -lmapper_interface -lgrid_mapping -lmap_io -lmap_util -lprob_models -lmap_interface \
	       -lmap_server_interface -lrotation_geometry -lgeometry -lm -lz -ltf \
	       -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt \
	       -lrt -lboost_signals -lboost_system \
	       -lsimulator_ackerman_interface -lnavigator_ackerman_interface -llocalize_ackerman_interface \
	       -lbehavior_selector_interface -lmapper_interface -lroute_planner_interface -lcollision_detection \
	       -lmoving_objects_interface
	       	
# Source code files (.c, .cpp) 
SOURCES = obstacle_distance_mapper_main.cpp obstacle_distance_mapper_datmo.cpp obstacle_distance_mapper_interface.c

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = obstacle_distance_mapper_messages.h obstacle_distance_mapper_interface.h obstacle_distance_mapper_datmo.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libobstacle_distance_mapper_interface.a libobstacle_distance_mapper_datmo.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = obstacle_distance_mapper

TARGETS = libobstacle_distance_mapper_interface.a libobstacle_distance_mapper_datmo.a obstacle_distance_mapper
PUBLIC_LIBRARIES_SO = libobstacle_distance_mapper_interface.so

libobstacle_distance_mapper_interface.a: obstacle_distance_mapper_interface.o

libobstacle_distance_mapper_datmo.a: obstacle_distance_mapper_datmo.o

libobstacle_distance_mapper_interface.so.1: obstacle_distance_mapper_interface.o

obstacle_distance_mapper: obstacle_distance_mapper_main.o obstacle_distance_mapper_datmo.o

# rules

include ../Makefile.rules

