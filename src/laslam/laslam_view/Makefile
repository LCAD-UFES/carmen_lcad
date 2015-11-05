include ../../Makefile.conf

# Module name and description
MODULE_NAME = LANDMARK SLAM - VIEW	
MODULE_COMMENT = Landmark Slam based on Visual Features 

LINK = g++

# Application specific include directories.
IFLAGS += -I/usr/local/include/bullet/ -I/usr/include/eigen3 -I$(CARMEN_HOME)/sharedlib/libtf/src

# Required default libraries to comunicate with Carmen Core
LFLAGS += -L/usr/local/lib -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -lparam_interface -lipc -lglobal -lcvis_interface -lglm -lGL -lGLU -lglut 	\
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
SOURCES += camera.cpp draw_callbacks.cpp gtk_gui.cpp drawer_3d.cpp robot.cpp visualodometry.cpp landmarks.cpp
PUBLIC_BINARIES +=
TARGETS += liblaslam_view.a
endif

liblaslam_view.a: camera.o draw_callbacks.o gtk_gui.o drawer_3d.o robot.o visualodometry.o landmarks.o

# rules
include ../../Makefile.rules
