include ../Makefile.conf

# Module name and description
MODULE_NAME = NEURAL SLAM - CORE
MODULE_COMMENT = Neural Slam based on Visual Saliencies
SUBDIRS += cvis_view

LINK = g++

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I/usr/include/eigen3

# Required default libraries to comunicate with Carmen Core
LFLAGS += -L/usr/local/lib -L/home/lcad/tplib/pcl/build/lib -L$(CARMEN_HOME)/sharedlib/libtf/src -L/usr/lib64
LFLAGS += -lparam_interface -lipc -lglobal -lmap_interface -lfused_odometry_interface -llocalize_ackerman_interface -lekf_odometry_interface -lbumblebee_basic_interface -lglm -ljpeg -lGL -lGLU -lglut -ltf -lBulletDynamics \
	  -lBulletCollision -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system \
	  -lopencv_core -lopencv_highgui -lopencv_ml -lopencv_imgproc

CFLAGS += -Wno-unknown-pragmas
CXXFLAGS += -Wno-unknown-pragmas

SUBDIRS = neural_slam_view

# Source code files (.c, .cpp) 
SOURCES = neural_slam_main.cpp neural_slam_dataset_associator.cpp neural_slam_server.cpp neural_slam_interface.c
 
# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = neural_slam_messages.h neural_slam_interface.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libneural_slam_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = neural_slam

TARGETS = libneural_slam_interface.a neural_slam neural_slam_dataset_associator
PUBLIC_LIBRARIES_SO = libneural_slam_interface.so

ifndef NO_PYTHON
TARGETS += libneural_slam_interface.so.1
endif

# If you set graphics support in configure.
ifndef NO_GRAPHICS
IFLAGS += -Ineural_slam_view/include
IFLAGS += -I/usr/include/libglade-2.0/

GLADE_FILES = neural_slam.glade

IFLAGS += `pkg-config --cflags gtk+-2.0 gmodule-export-2.0`
IFLAGS += `pkg-config --cflags gtkglext-1.0`
LFLAGS += `pkg-config --libs gtk+-2.0 gmodule-export-2.0`
LFLAGS += `pkg-config --libs gtkglext-1.0`

#Carmen Global Graphic Library and Gtk+2 Graphics Libraries
LFLAGS += -lglobal_graphics -lgdk-x11-2.0 -lgdk_pixbuf-2.0
endif

libneural_slam_interface.a:	neural_slam_interface.o

libneural_slam_interface.so.1:	neural_slam_interface.o

neural_slam: neural_slam_main.o neural_slam_server.o libneural_slam_interface.a neural_slam_view/libneural_slam_view.a

neural_slam_dataset_associator: neural_slam_dataset_associator.o

# rules

include ../Makefile.rules
