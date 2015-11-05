include ../../Makefile.conf

# Module name and description
MODULE_NAME = CVIS_VIEW
MODULE_COMMENT = Carmen Visualization Tool

LINK = g++

# Application specific include directories.
IFLAGS += -I/usr/local/include/bullet/ -I/usr/include/eigen3 -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/cuda/include -I/home/lcad/NVIDIA_GPU_Computing_SDK/C/common/inc
IFLAGS += -I/home/lcad/tplib/pcl/build/include -I/home/lcad/tplib/pcl/trunk/common/include \

# Required default libraries to comunicate with Carmen Core
LFLAGS += -L/usr/local/lib -L/home/lcad/tplib/pcl/build/lib -L$(CARMEN_HOME)/sharedlib/libtf/src -L/usr/local/cuda/lib64/lib
LFLAGS += -lparam_interface -lipc -lglobal -lcvis_interface -lstereo_interface -lvelodyne_interface -lfused_odometry_interface -lGL -lGLU -lglut\
		  -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system \
		  `pkg-config --libs opencv`
CFLAGS += -Wno-unknown-pragmas
CXXFLAGS += -Wno-unknown-pragmas

# If you set graphics support in configure.
ifndef NO_GRAPHICS
IFLAGS += -Iinclude/
IFLAGS += -I/usr/include/libglade-2.0/
IFLAGS += `pkg-config --cflags gtk+-2.0 gmodule-export-2.0`
IFLAGS += `pkg-config --cflags gtkglext-1.0`
LFLAGS += `pkg-config --libs gtk+-2.0 gmodule-export-2.0`
LFLAGS += `pkg-config --libs gtkglext-1.0`

#Carmen Global Graphic Library and Gtk+2 Graphics Libraries
LFLAGS += -lglobal_graphics
#Source code file (.c)
SOURCES += camera.cpp draw_callbacks.cpp gtk_gui.cpp drawer_3d.cpp point_cloud.cpp vertex_buffer_objects.cpp velodyne.cpp kinect.cpp #slam6d.cpp
PUBLIC_BINARIES +=
TARGETS += libcvis_view.a
endif

libcvis_view.a: camera.o draw_callbacks.o gtk_gui.o drawer_3d.o point_cloud.o vertex_buffer_objects.o velodyne.o kinect.o #slam6d.o

# rules
include ../../Makefile.rules
