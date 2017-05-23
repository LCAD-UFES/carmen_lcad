include ../Makefile.conf

MODULE_NAME = Carmen Velodyne to Map
MODULE_COMMENT = Carmen Velodyne to Map

LINK = g++

CFLAGS += -Wall -std=c99
CXXFLAGS += -Wall -std=c++98

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -lglobal -lipc -lvelodyne_interface -lparam_interface `pkg-config --libs opencv` \
-L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lrt -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
		-lboost_thread-mt -lboost_signals -lboost_system

SOURCES = carmen_velodyne_to_map_main.cpp box_positions_to_pixels.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 

TARGETS = carmen_velodyne_to_map box_positions_to_pixels

carmen_velodyne_to_map: carmen_velodyne_to_map_main.o 
box_positions_to_pixels: box_positions_to_pixels.o

include ../Makefile.rules
