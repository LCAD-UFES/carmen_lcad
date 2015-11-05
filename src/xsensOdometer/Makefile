include ../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += -lglobal -lipc -lbase_interface -llocalize_interface -lparam_interface -lxsens_interface

MODULE_NAME = xsensOdometer
MODULE_COMMENT = This module creates odometry messages based on the xsens sendor messages. 

SOURCES = xsensOdometer.c xsensOdometer_interface.c
PUBLIC_INCLUDES = xsensOdometer_messages.h xsensOdometer_interface.h

PUBLIC_LIBRARIES = libxsensOdometer_interface.a
PUBLIC_BINARIES = xsensOdometer
MAN_PAGES =

TARGETS = libxsensOdometer_interface.a xsensOdometer 

PUBLIC_LIBRARIES_SO = libxsensOdometer_interface.so

ifndef NO_PYTHON
TARGETS += libxsensOdometer_interface.so.1
endif

# rules
libxsensOdometer_interface.a:	xsensOdometer_interface.o

libxsensOdometer_interface.so.1:	xsensOdometer_interface.o

xsensOdometer:	xsensOdometer.o libxsensOdometer_interface.a

include ../Makefile.rules



