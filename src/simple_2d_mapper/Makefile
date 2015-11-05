include ../Makefile.conf

# Module name and description
MODULE_NAME = SIMPLE_2D_MAPPER
MODULE_COMMENT = Simple 2D Mapper Package

LINK = g++

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src 
LFLAGS += -lparam_interface -lipc -lglobal -lvelodyne_interface -lfused_odometry_interface -ltf \
		  -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lboost_system
CFLAGS += -Wall
# Source code files (.c, .cpp) 
SOURCES = simple_2d_mapper_main.cpp simple_2d_mapper_core.cpp simple_2d_mapper_interface.c

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = simple_2d_mapper_interface.h simple_2d_mapper_messages.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libsimple_2d_mapper_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = simple_2d_mapper

TARGETS = libsimple_2d_mapper_interface.a simple_2d_mapper
PUBLIC_LIBRARIES_SO = libsimple_2d_mapper_interface.so

ifndef NO_PYTHON
TARGETS += libsimple_2d_mapper_interface.so.1
endif

# If you set graphics support in configure (only for GTK).
ifndef NO_GRAPHICS
FLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += `pkg-config --libs gtk+-2.0`
#Source code file (.c)
TARGETS += 
SOURCES += 
endif

libsimple_2d_mapper_interface.a:	simple_2d_mapper_interface.o

simple_2d_mapper:	simple_2d_mapper_main.o simple_2d_mapper_core.o libsimple_2d_mapper_interface.a  

# rules
include ../Makefile.rules
