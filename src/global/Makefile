include ../Makefile.conf

LINK=g++

LFLAGS += -lipc -lparam_interface -lstdc++

ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags`
LFLAGS += `$(GTK_CONFIG) --libs`
IFLAGS += -I. -I/usr/include/x86_64-linux-gnu/qt5/QtCore/ -I/usr/include/x86_64-linux-gnu/qt5/QtGui -I/usr/include -I/usr/include/x86_64-linux-gnu/qt5 -I$CARMEN_HOME/include
# IFLAGS += -I/usr/lib64/qt4/mkspecs/linux-g++ -I. -I/usr/include/qt4/QtCore/ -I/usr/include/qt4/QtGui -I/usr/include -I/usr/include/qt4/ -I$CARMEN_HOME/include
# IFLAGS += -I/usr/lib64/qt4/mkspecs/linux-g++ -I. -I/usr/include/QtCore -I/usr/include/QtGui -I/usr/include 
LFLAGS += -lQt5Gui -lQt5Core -lpthread
else
CFLAGS += -DNO_GRAPHICS
endif

CFLAGS += -Wno-deprecated-declarations
CXXFLAGS += -Wno-deprecated-copy

MODULE_NAME = "GLOBAL"
MODULE_COMMENT = "CARMEN global functions"

SOURCES = global.c carmen_stdio.c geometry.c pswrap.c carmenserial.c global_test.c \
	  keyctrl.c multicentral.c test_multicentral.c \
	  ipc_wrapper.c movement.c test_movement.c \
	  carmen_gps.cpp Gdc_Coord_3d.cpp Gdc_To_Utm_Converter.cpp Utm_Coord_3d.cpp Utm_To_Gdc_Converter.cpp carmen_gps_wrapper.cpp test_serial.c parking_sensor_serial.c

PUBLIC_INCLUDES = global.h carmen_stdio.h ipc_wrapper.h geometry.h pswrap.h \
	  	  carmen.h carmenserial.h keyctrl.h multicentral.h movement.h \
		  carmen_gps.h Gdc_Coord_3d.h Gdc_To_Utm_Converter.h Utm_Coord_3d.h Utm_To_Gdc_Converter.h carmen_gps_wrapper.h

PUBLIC_LIBRARIES = libglobal.a libgeometry.a libpswrap.a libcarmenserial.a \
		   libkeyctrl.a libmulticentral.a libmovement.a libcarmen_gps_wrapper.a

PUBLIC_BINARIES = carmen-config 

TARGETS = libglobal.a libgeometry.a libpswrap.a libcarmenserial.a global_test \
	  libkeyctrl.a libmulticentral.a \
	  test_multicentral libmovement.a test_movement  libcarmen_gps_wrapper.a test_serial parking_sensor_serial

CHECK_CONFIG = $(shell if [ -f carmen-config.c ]; then echo "1"; fi;)

ifneq ($(CHECK_CONFIG),1)
error:
	$(ECHO) "I can't find carmen-configure.c. Did you remember to run\
(or re-run) configure?"
endif

PUBLIC_LIBRARIES_SO = libglobal.so
ifndef NO_PYTHON
TARGETS += libglobal.so.1
endif

ifndef NO_GRAPHICS
SOURCES += global_graphics.c global_graphics_qt.cpp
PUBLIC_INCLUDES += global_graphics.h global_graphics_qt.h carmen_graphics.h 
PUBLIC_LIBRARIES += libglobal_graphics.a libglobal_graphics_qt.a
TARGETS += libglobal_graphics.a libglobal_graphics_qt.a
endif

ifndef NO_LIBJPEG
SOURCES += global_jpeg.c
CFLAGS += -DHAVE_JPEG
endif

ifndef NO_LIBART
SOURCES += artwrap.c
PUBLIC_INCLUDES += artwrap.h
PUBLIC_LIBRARIES += libartwrap.a 
TARGETS += libartwrap.a
CFLAGS += -DHAVE_LIBART
endif

libglobal.a:		global.o ipc_wrapper.o carmen_stdio.o carmen_gps.o Gdc_To_Utm_Converter.o Utm_To_Gdc_Converter.o Utm_Coord_3d.o Gdc_Coord_3d.o

libglobal.so.1:		global.o ipc_wrapper.o carmen_stdio.o


ifndef NO_LIBJPEG
libglobal_graphics.a:	global_graphics.o global_jpeg.o ../road_mapper/road_mapper.o
libglobal_graphics_qt.a:	global_graphics_qt.o global_jpeg.o
else
libglobal_graphics.a:	global_graphics.o ../road_mapper/road_mapper.o
libglobal_graphics_qt.a:  global_graphics_qt.o
endif

libgeometry.a:		geometry.o

libpswrap.a:		pswrap.o

libartwrap.a:		artwrap.o

libcarmenserial.a:	carmenserial.o

libkeyctrl.a:   	keyctrl.o

libmulticentral.a:	multicentral.o 

test_multicentral:	test_multicentral.o libmulticentral.a libglobal.a 

global_test: 		global_test.o libglobal.a

carmen-config: 		carmen-config.o 

libmovement.a: movement.o

test_movement: movement.o test_movement.o libglobal.a

test_serial: test_serial.o carmenserial.o

parking_sensor_serial: parking_sensor_serial.o carmenserial.o

libcarmen_gps_wrapper.a: carmen_gps_wrapper.o Utm_Coord_3d.o Utm_To_Gdc_Converter.o Gdc_To_Utm_Converter.o Gdc_Coord_3d.o  

include ../Makefile.rules
