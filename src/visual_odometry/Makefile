include ../Makefile.conf

MODULE_NAME = VISUAL_ODOMETRY
MODULE_COMMENT = Visual Odometry Package

LINK = g++
CXXFLAGS += -Wall -msse3

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libviso2.3/src -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libviso2.3/src -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -lparam_interface -lipc -lglobal -lbumblebee_basic_interface -lvisual_odometry_interface -lstereo_interface -lviso
LFLAGS += -lrotation_geometry -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system `pkg-config --libs opencv`

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I/usr/include/bullet/ -I/usr/local/carmen_boost/include
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L/usr/local/carmen_boost/lib

SOURCES = visual_odometry_main.cpp visual_odometry_interface.c
PUBLIC_INCLUDES = visual_odometry_messages.h visual_odometry_interface.h

PUBLIC_LIBRARIES = libvisual_odometry_interface.a
PUBLIC_BINARIES = visual_odometry
PUBLIC_LIBRARIES_SO = libvisual_odometry_interface.so

TARGETS = libviso visual_odometry libvisual_odometry_interface.a

ifndef NO_PYTHON
TARGETS += libvisual_odometry_interface.so.1
endif

ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags`
IFLGAS += `pkg-config --cflags opencv`  
LFLAGS += -lglobal_graphics `$(GTK_CONFIG) --libs`
LFLAGS += `pkg-config --libs opencv`  
SOURCES +=
PUBLIC_BINARIES +=
TARGETS +=
endif

visual_odometry:	visual_odometry_main.o visual_odometry_interface.a	

libvisual_odometry_interface.a:	visual_odometry_interface.o

libvisual_odometry_interface.so.1:	visual_odometry_interface.o

libviso:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libviso2.3/src

clean: clean_libviso

clean_libviso:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libviso2.3/src
# rules
include ../Makefile.rules
