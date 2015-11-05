include ../Makefile.conf

LINK=g++
LFLAGS += -lparam_interface -lglobal -lipc -lm


MODULE_NAME = MAPTOOLS
MODULE_COMMENT = Tools for mapping etc.

SOURCES = bee2carmen.c generate_blank.c map.c \
	linemapping.cpp map_interface.c map_ipc.c map_read.c \
	maptool.c map_util.c map_write.c

PUBLIC_INCLUDES = linemapping.h map.h map_io.h map_messages.h map_interface.h \
	 map_util.h
PUBLIC_LIBRARIES = liblinemapping.a libmap_interface.a  \
	libmap_util.a libmap_io.a
PUBLIC_BINARIES = clf2linemap map maptool

TARGETS = map libmap_interface.a   libmap_util.a \
	libmap_io.a maptool


PUBLIC_LIBRARIES_SO = libmap_interface.so
ifndef NO_PYTHON
TARGETS += libmap_interface.so.1
endif

# liblinemapping.a clf2linemap
ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags`
LFLAGS += -lglobal_graphics `$(GTK_CONFIG) --libs`
SOURCES += map_graphics.c map_test.c 
PUBLIC_INCLUDES += map_graphics.h
PUBLIC_LIBRARIES += libmap_graphics.a 
TARGETS += libmap_graphics.a map_test
else
 CFLAGS += -DNO_GRAPHICS
endif

all:

libmap_io.a: map_read.o map_write.o map_ipc.o

map: map.o libmap_interface.a libmap_io.a 

map_test:       map_test.o libmap_interface.a libmap_graphics.a

libmap_interface.a: map_interface.o

libmap_interface.so.1: map_interface.o

libmap_graphics.a:	map_graphics.o libmap_interface.a

liblinemapping.a:    linemapping.o

clf2linemap:         clf2linemap.o clfreader.o linemapping.o

libmap_util.a: map_util.o

maptool: maptool.o libmap_util.a libmap_io.a libmap_interface.a

generate_blank : generate_blank.o libmap_io.a

bee2carmen : bee2carmen.o libmap_io.a

include  ../Makefile.rules

