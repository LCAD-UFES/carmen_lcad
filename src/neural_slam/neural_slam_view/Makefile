include ../../Makefile.conf

# Module name and description
MODULE_NAME = NEURAL SLAM - VIEW	
MODULE_COMMENT = Neural Slam based on Visual Saliencies 

LINK = g++

# Application specific include directories.
IFLAGS += -I/usr/local/include/bullet/ -I/usr/include/eigen3 -I$(CARMEN_HOME)/sharedlib/libtf/src

# Required default libraries to comunicate with Carmen Core
LFLAGS += -L/usr/local/lib -L$(CARMEN_HOME)/sharedlib/libtf/src -L/usr/lib64
LFLAGS += -lparam_interface -lipc -lglobal -lglm -lGL -lGLU -lglut 	\
	  -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system \
	  -lopencv_core -lopencv_highgui -lopencv_ml -lopencv_imgproc

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
SOURCES += camera.cpp draw_callbacks.cpp gtk_gui.cpp drawer_3d.cpp robot.cpp odometry.cpp saliency.cpp
PUBLIC_BINARIES +=
TARGETS += libneural_slam_view.a
endif

libneural_slam_view.a: camera.o draw_callbacks.o gtk_gui.o drawer_3d.o robot.o odometry.o saliency.o

# rules
include ../../Makefile.rules
