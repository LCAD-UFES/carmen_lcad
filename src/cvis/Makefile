include ../Makefile.conf

# Module name and description
MODULE_NAME = CVIS
MODULE_COMMENT = Carmen Visualization Tool
SUBDIRS += cvis_view

LINK = g++

# Application specific include directories.
IFLAGS += -I/usr/local/include/bullet/ -I/usr/include/eigen3 -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/cuda/include -I/home/lcad/NVIDIA_GPU_Computing_SDK/C/common/inc

PCL_INC = $(wildcard /usr/local/include/pcl-*)
VTK_INC = $(wildcard /usr/local/include/vtk-5* /usr/include/vtk-5*)
IFLAGS += -I/usr/include/eigen3 -I $(PCL_INC) -I $(VTK_INC)

# Required default libraries to comunicate with Carmen Core
LFLAGS += -L/usr/local/lib -L$(CARMEN_HOME)/sharedlib/libtf/src -L/usr/local/cuda/lib64/lib
LFLAGS += -lparam_interface -lipc -lglobal -lcvis_interface -lstereo_interface -lvelodyne_interface -lkinect_interface -lfused_odometry_interface -ljoystick_interface -lGL -lGLU -lglut\
		  -ltf -lBulletDynamics -lBulletCollision -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system \
		  `pkg-config --libs opencv`
CFLAGS += -Wno-unknown-pragmas
CXXFLAGS += -Wno-unknown-pragmas
CUDAFLAGS += -I$(CARMEN_HOME)/include

# Source code files (.c, .cpp) 
SOURCES = cvis_main.cpp cvis_interface.c #kernel_vbo.cu

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = cvis_messages.h cvis_interface.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libcvis_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = cvis

TARGETS = libcvis_interface.a cvis
PUBLIC_LIBRARIES_SO = libcvis_interface.so

ifndef NO_PYTHON
TARGETS += libcvis_interface.so.1
endif

# If you set graphics support in configure.
ifndef NO_GRAPHICS
IFLAGS += -Icvis_view/include
IFLAGS += -I/usr/include/libglade-2.0/
IFLAGS += `pkg-config --cflags gtk+-2.0 gmodule-export-2.0`
IFLAGS += `pkg-config --cflags gtkglext-1.0`
LFLAGS += `pkg-config --libs gtk+-2.0 gmodule-export-2.0`
LFLAGS += `pkg-config --libs gtkglext-1.0`
#Carmen Global Graphic Library and Gtk+2 Graphics Libraries
LFLAGS += -lglobal_graphics -lgdk-x11-2.0 -lgdk_pixbuf-2.0
endif

libcvis_interface.a:	cvis_interface.o

libcvis_interface.so.1:	cvis_interface.o

cvis: cvis_main.o  libcvis_interface.a cvis_view/libcvis_view.a #kernel_vbo.o 

# rules

include ../Makefile.rules
