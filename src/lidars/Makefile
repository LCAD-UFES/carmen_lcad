include ../Makefile.conf

MODULE_NAME = VELODYNE2
MODULE_COMMENT = Velodyne2 LIDAR Module for variable_message

LINK = g++

CFLAGS += -std=c11
CXXFLAGS += -std=c++11

IFLAGS += -I$(CARMEN_HOME)/sharedlib/librobosense
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libvelodyne2
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libvelodyne2
LFLAGS += -L$(CARMEN_HOME)/sharedlib/librobosense
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src

LFLAGS += -lparam_interface -lipc -lglobal -lvelodyne_interface -lvelodyne2 -lrobosense
LFLAGS += -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system -pthread
CFLAGS += -Wall

SOURCES = lidars_main.cpp robosense_RSLiDAR16.cpp

#PUBLIC_INCLUDES =

#PUBLIC_LIBRARIES =

PUBLIC_BINARIES = lidars

TARGETS = lidars libvelodyne2.a librobosense.a

#PUBLIC_LIBRARIES_SO = 

librobosense.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/librobosense 
	
libvelodyne2.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libvelodyne2 

lidars: lidars_main.o robosense_RSLiDAR16.o

clean: clean_libvelodyne2 clean_librobosense

clean_libvelodyne2:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libvelodyne2/
	
clean_librobosense:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/librobosense/

include ../Makefile.rules
