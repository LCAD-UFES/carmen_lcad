include ../Makefile.conf

MODULE_NAME = LIDARS
MODULE_COMMENT = Velodyne2 LIDAR Module for variable_message

LINK = g++

CFLAGS += -std=c11
CXXFLAGS += -std=c++11

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libvelodyne2
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I/usr/local/carmen_boost/include
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libvelodyne2 -L/usr/local/carmen_boost/lib
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src

LFLAGS += -lparam_interface -lipc -lglobal -lvelodyne_interface -lvelodyne2 -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody \
          -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system -pthread

SOURCES = lidars_main.cpp robosense_RSLiDAR16.cpp

#PUBLIC_INCLUDES =

#PUBLIC_LIBRARIES =

PUBLIC_BINARIES = lidars

TARGETS = lidars libvelodyne2.a librobosense.a

#PUBLIC_LIBRARIES_SO = 

libvelodyne2.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libvelodyne2 

lidars: lidars_main.o robosense_RSLiDAR16.o

clean: clean_libvelodyne2

clean_libvelodyne2:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libvelodyne2/
	
include ../Makefile.rules
