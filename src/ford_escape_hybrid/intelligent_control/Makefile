include ../../Makefile.conf

LFLAGS += -lglobal -lgeometry -lparam_interface -lipc

CFLAGS += -std=c99

MODULE_NAME = INTELLIGENT_CONTROL
MODULE_COMMENT = Module improving car control commands

SOURCES = intelligent_control.c

PUBLIC_BINARIES = intelligent_control 

TARGETS = intelligent_control

intelligent_control: intelligent_control.o

include ../../Makefile.rules
