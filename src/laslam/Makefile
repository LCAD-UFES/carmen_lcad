include ../Makefile.conf

# Module name and description
MODULE_NAME = LANDMARK SLAM - CORE
MODULE_COMMENT = Landmark Slam based on Visual Features

LINK = g++

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I/usr/include/eigen3

# Required default libraries to comunicate with Carmen Core
LFLAGS += -L/usr/local/lib -L/home/lcad/tplib/pcl/build/lib -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -lparam_interface -lipc -lglobal -lstereo_interface -lbumblebee_basic_interface -lvisual_odometry_interface \
	  -lglm -ljpeg -lGL -lGLU -lglut -ltf -lBulletDynamics \
	  -lBulletCollision -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system \
	  `pkg-config --libs opencv`

CFLAGS += -Wno-unknown-pragmas
CXXFLAGS += -Wno-unknown-pragmas

SUBDIRS = laslam_view

# Source code files (.c, .cpp) 
SOURCES = laslam_main.cpp laslam_server.cpp laslam_interface.c laslam_features.cpp laslam_interpolation.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = laslam_messages.h laslam_interface.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = liblaslam_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = laslam

TARGETS = liblaslam_interface.a laslam
PUBLIC_LIBRARIES_SO = liblaslam_interface.so

ifndef NO_PYTHON
TARGETS += liblaslam_interface.so.1
endif

# If you set graphics support in configure.
ifndef NO_GRAPHICS
IFLAGS += -Ilaslam_view/include
IFLAGS += -I/usr/include/libglade-2.0/

GLADE_FILES = laslam.glade

IFLAGS += `pkg-config --cflags gtk+-2.0 gmodule-export-2.0`
IFLAGS += `pkg-config --cflags gtkglext-1.0`
LFLAGS += `pkg-config --libs gtk+-2.0 gmodule-export-2.0`
LFLAGS += `pkg-config --libs gtkglext-1.0`

#Carmen Global Graphic Library and Gtk+2 Graphics Libraries
LFLAGS += -lglobal_graphics -lgdk-x11-2.0 -lgdk_pixbuf-2.0
endif

liblaslam_interface.a:	laslam_interface.o

liblaslam_interface.so.1:	laslam_interface.o

laslam: laslam_main.o laslam_server.o laslam_features.o laslam_interpolation.o liblaslam_interface.a laslam_view/liblaslam_view.a

# rules

include ../Makefile.rules
