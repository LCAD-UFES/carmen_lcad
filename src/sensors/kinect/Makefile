include $(CARMEN_HOME)/src/Makefile.conf

LINK=g++
CFLAGS += -std=c99
IFLAGS += -I/usr/local/include/libfreenect 
LFLAGS += -lglobal -lparam_interface -lipc -lfreenect -llaser_interface -lGL -lGLU -lglut

MODULE_NAME = KINECT
MODULE_COMMENT = Kinect Data 

SOURCES = kinect.c kinect_mrds.c kinect_laser.c kinect_interface.c kinect_util.c
PUBLIC_INCLUDES = kinect_messages.h kinect_interface.h kinect_util.h

PUBLIC_LIBRARIES = libkinect_interface.a
PUBLIC_BINARIES = kinect kinect_mrds kinect_laser
MAN_PAGES =

TARGETS = kinect kinect_mrds kinect_laser kinect_wrapper.a libkinect_interface.a

PUBLIC_LIBRARIES_SO = 

ifndef NO_PYTHON
TARGETS += 
endif

#NO_GRAPHICS=1
ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += -lparam_interface -lipc -lglobal -lglobal_graphics \
	`$(GTK_CONFIG) --libs` 
SOURCES += kinect_view.c
PUBLIC_BINARIES += kinect_view
TARGETS += kinect_view
endif

ifdef CARMEN_DEBUG
CFLAGS += -DDEBUG
endif

kinect_wrapper.a:
	$(MAKE) -C kinect_wrapper

libkinect_interface.a: kinect_interface.o kinect_util.o

kinect: kinect.o kinect_wrapper.a libkinect_interface.a

kinect_mrds: kinect_mrds.o libkinect_interface.a

kinect_view: kinect_view.o libkinect_interface.a

kinect_laser: kinect_laser.o libkinect_interface.a

include $(CARMEN_HOME)/src/Makefile.rules

clean: clean_kinect_wrapper

clean_kinect_wrapper:
	$(MAKE) clean -C kinect_wrapper
