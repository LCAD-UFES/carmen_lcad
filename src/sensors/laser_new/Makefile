include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = "LASER NEW"
MODULE_COMMENT = "The new laser module for SICK and Hokuyo lasers"

CFLAGS += -DLASER_USE_PTHREAD -std=gnu99 
#-DHOKUYO_ALWAYS_IN_SCIP20
LFLAGS += -lparam_interface -lcarmenserial -lglobal -lipc -lpthread -lX11 -lXext

ifeq (Fedora release 14 (Laughlin),$(SO_DISTRO))
  LFLAGS += -lX11 -lXext
endif


SOURCES = carmen_hokuyo.c  dummylaser.c hokuyourg_test.c laser_interface.c sick_laser_init_500k.c \
          carmen_laser_device.c carmen_laser_message_queue.c laser.c sick_laser_test.c \
          carmen_laser_device_init.c carmen_sick.c hokuyourg.c laserclient.c sick_laser.c

SOURCES += s300_laser.c carmen_s300.c

PUBLIC_INCLUDES = laser_messages.h laser_interface.h
PUBLIC_LIBRARIES = liblaser_interface.a liblaser.a
PUBLIC_BINARIES = laser sick_laser_init_500k

TARGETS =  liblaser_interface.a hokuyourg_test sick_laser_test dummylaser laserclient laser  sick_laser_init_500k
TARGETS += s300_laser_test

ifndef NO_TCP862_SERIAL_SUPPORT
CFLAGS += -DUSE_TCP862 
LFLAGS += -ltcp862
endif

ifndef NO_GRAPHICS
ifndef NO_LIBJPEG
CFLAGS +=
IFLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += -lparam_interface -lipc -lglobal -lglobal_graphics  \
	`$(GTK_CONFIG) --libs` -lmap_graphics -ljpeg 
SOURCES += laserview.c
PUBLIC_BINARIES += laserview
TARGETS += laserview
endif
endif


PUBLIC_LIBRARIES_SO = liblaser_interface.so
ifndef NO_PYTHON
TARGETS += liblaser_interface.so.1
liblaser_interface.so.1: laser_interface.o carmen_laser_device.o
endif

liblaser_interface.a:	laser_interface.o carmen_laser_device.o

liblaser.a:		carmen_laser_device.o carmen_laser_message_queue.o hokuyourg.o\
			carmen_hokuyo.o carmen_laser_device_init.o  sick_laser.o carmen_sick.o \
			s300_laser.o carmen_s300.o 

hokuyourg_test:		hokuyourg_test.o liblaser.a

sick_laser_test:	sick_laser_test.o liblaser.a

s300_laser_test:	s300_laser_test.o liblaser.a

sick_laser_init_500k:	sick_laser_init_500k.o liblaser.a

laser:                  laser.o liblaser_interface.a liblaser.a

dummylaser:             dummylaser.o liblaser_interface.a liblaser.a

laserview:              laserview.o liblaser_interface.a 

laserclient:            laserclient.o liblaser_interface.a 



include $(CARMEN_HOME)/src/Makefile.rules
