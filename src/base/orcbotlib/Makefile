include ../../Makefile.conf

SUBDIRS = orclib_v5

ifndef CYGWIN
CFLAGS += -fPIC 
endif
CFLAGS += -std=gnu99 -g
IFLAGS +=
LFLAGS += -lglobal -lipc -lparam_interface -lorcbot -lcarmenserial #-lbase_interface 

MODULE_NAME = "ORC_V5"
MODULE_COMMENT = "Library for controlling ORC (v5) robots"

SOURCES = orcarm_test.c orcdrive_test.c orc_arm_lib.c orc_drive_lib.c

PUBLIC_LIBRARIES = liborcbot.a
PUBLIC_BINARIES = 
MAN_PAGES = 

TARGETS = liborcbot.a orcarm_test orcdrive_test

liborcbot.a: orc_drive_lib.o orc_arm_lib.o orclib_v5/liborc.a

orcarm_test: orcarm_test.o orc_drive_lib.o orc_arm_lib.o orclib_v5/liborc.a

orcdrive_test: orcdrive_test.o orc_drive_lib.o orc_arm_lib.o orclib_v5/liborc.a

include ../../Makefile.rules
