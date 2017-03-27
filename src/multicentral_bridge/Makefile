include ../Makefile.conf

LINK=g++

IFLAGS += -I$CARMEN_HOME/include

LFLAGS += -lipc -lglobal -lglobal_graphics -lmulticentral -lparam_interface -lstdc++ \
		-llocalize_ackerman_interface -lbumblebee_basic_interface -ldc1394 -lrddf_interface
#		`pkg-config --libs opencv` -lflycapture \
#		-lGL -lGLU -lglut -ltime_profile \
#		`pkg-config --libs opencv` \
		

#ifndef NO_GRAPHICS
#IFLAGS += `$(GTK_CONFIG) --cflags`
#LFLAGS += `$(GTK_CONFIG) --libs`
#IFLAGS += -I/usr/lib64/qt4/mkspecs/linux-g++ -I. -I/usr/include/qt4/QtCore/ -I/usr/include/qt4/QtGui -I/usr/include -I/usr/include/qt4/ -I$CARMEN_HOME/include
#IFLAGS += -I/usr/lib64/qt4/mkspecs/linux-g++ -I. -I/usr/include/QtCore -I/usr/include/QtGui -I/usr/include 
#LFLAGS += -lQtGui -lQtCore -lpthread
#else
#CFLAGS += -DNO_GRAPHICS
#endif

MODULE_NAME = "MULTICENTRAL_BRIDGE"
MODULE_COMMENT = "CARMEN MULTICENTRAL_BRIDGE module to comunicate with centrals in other robots"

SOURCES = multicentral_bridge_traffic_light.c test_bridge_bumblebee.c

#PUBLIC_INCLUDES = 

#PUBLIC_LIBRARIES = 

#PUBLIC_BINARIES =  

TARGETS = bridge_traffic_light test_bridge_bumblebee

#PUBLIC_LIBRARIES_SO =

#libmulticentral.a:	multicentral.o 

test_bridge_bumblebee: test_bridge_bumblebee.o
bridge_traffic_light: multicentral_bridge_traffic_light.o

include ../Makefile.rules