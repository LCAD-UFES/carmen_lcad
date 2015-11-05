include ../Makefile.conf

MODULE_NAME = "LASER"
MODULE_COMMENT = "SICK Laser Module"

CFLAGS +=
IFLAGS += 
LFLAGS += -lparam_interface -lcarmenserial -lipc -lglobal
SOURCES = sick.c laser_ipc.c laser_main.c laser.c laser_test.c
PUBLIC_INCLUDES = laser_messages.h laser_interface.h
PUBLIC_LIBRARIES = liblaser_interface.a  liblaser.a
PUBLIC_BINARIES = laser
TARGETS = laser laser_test liblaser_interface.a  liblaser.a

ifndef NO_GRAPHICS
ifndef NO_LIBJPEG
CFLAGS +=
IFLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += -lparam_interface -lipc -lglobal -lglobal_graphics \
	`$(GTK_CONFIG) --libs` -lmap_graphics -ljpeg
SOURCES += laserview.c
PUBLIC_INCLUDES += 
PUBLIC_LIBRARIES += 
PUBLIC_BINARIES += laserview
TARGETS += laserview
endif
endif

PUBLIC_LIBRARIES_SO = liblaser_interface.so
ifndef NO_PYTHON
TARGETS += liblaser_interface.so.1
endif

laser:			sick.o laser_ipc.o laser_main.o laser.o

laser_test:		laser_test.o liblaser_interface.a

liblaser.a:		sick.o laser_ipc.o laser_main.o

liblaser_interface.a:	laser_interface.o

liblaser_interface.so.1: laser_interface.o

laserview:		laserview.o liblaser_interface.a

# rules

include ../Makefile.rules
