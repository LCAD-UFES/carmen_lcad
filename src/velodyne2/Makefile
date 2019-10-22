include ../Makefile.conf

# Module name and description
MODULE_NAME = VELODYNE2
MODULE_COMMENT = Velodyne LIDAR Module

LINK = g++

CFLAGS += -std=c11
CXXFLAGS += -std=c++11

# Application specific include directories.

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libvelodyne2
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libvelodyne2
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lipc -lglobal -lvelodyne_interface -lvelodyne -lrotation_geometry
LFLAGS += -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system -pthread
CFLAGS += -Wall

# Source code files (.c, .cpp) 
SOURCES = velodyne_main.cpp velodyne_interface.cpp velodyne_transforms.cpp tf_helper.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = velodyne_messages.h velodyne_interface.h velodyne_mean_error.h velodyne_transforms.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libvelodyne_interface.a libvelodyne_transforms.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = velodyne

TARGETS = libvelodyne_interface.a velodyne libvelodyne.a libvelodyne_transforms.a
PUBLIC_LIBRARIES_SO = libvelodyne_interface.so libvelodyne_transforms.so

ifndef NO_PYTHON
TARGETS += libvelodyne_interface.so.1
endif

# If you set graphics support in configure (only for GTK).
ifndef NO_GRAPHICS
FLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += `pkg-config --libs gtk+-2.0`
#Source code file (.c)
TARGETS +=
SOURCES += 
endif

libvelodyne.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libvelodyne2 
libvelodyne_transforms.a: velodyne_transforms.o tf_helper.o
libvelodyne_interface.a: velodyne_interface.o
#libvelodyne_interface.a: velodyne_interface.o tf_helper.o

libvelodyne_interface.so.1:	velodyne_interface.o 
libvelodyne_transforms.so.1:  velodyne_transforms.o tf_helper.o

velodyne: velodyne_main.o libvelodyne_interface.a 

clean: clean_libvelodyne2

clean_libvelodyne2:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libvelodyne2/


# rules

include ../Makefile.rules
