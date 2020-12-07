include ../Makefile.conf

MODULE_NAME = VISUAL_TRACKER
MODULE_COMMENT = Visual Tracker Module

CC = g++
LINK = g++

IFLAGS += -I/usr/include/QtCore -I/usr/include/QtGui
IFLAGS += -I./libtld/include
IFLAGS += `pkg-config --cflags opencv` #-I$(OPENCV_HOME)/include
LFLAGS += `pkg-config --libs opencv` #-L$(OPENCV_HOME)/lib
LFLAGS += -L./libtld
LFLAGS += -lparam_interface -lipc -lglobal -lbumblebee_basic_interface
LFLAGS += -lopencv_video -lopencv_imgproc #-lopencv_core -lopencv_highgui
LFLAGS += -lQtGui -lQtCore -ltld

SOURCES = visual_tracker_interface.c visual_tracker_util.c visual_tracker_bumblebee.c visual_tracker_main.cpp

PUBLIC_INCLUDES = visual_tracker_interface.h visual_tracker_messages.h visual_tracker_util.h
PUBLIC_LIBRARIES = libvisual_tracker_interface.a 
PUBLIC_BINARIES = visual_tracker visual_tracker_bumblebee 

TARGETS = libvisual_tracker_interface.a #libtld.a libvisual_tracker_interface.a visual_tracker visual_tracker_bumblebee

PUBLIC_LIBRARIES_SO = libvisual_tracker_interface.so

ifndef NO_PYTHON
TARGETS += libvisual_tracker_interface.so.1
endif

libtld.a:
	$(MAKE) -C libtld

libvisual_tracker_interface.a:		visual_tracker_interface.o visual_tracker_util.o 

libvisual_tracker_interface.so.1:	visual_tracker_interface.o visual_tracker_util.o 

visual_tracker:	visual_tracker_main.o libvisual_tracker_interface.a

visual_tracker_bumblebee: visual_tracker_bumblebee.o libvisual_tracker_interface.a

clean: clean_visual_tracker

clean_visual_tracker:
	$(MAKE) clean -C libtld

# rules

include ../Makefile.rules

