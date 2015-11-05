include ../Makefile.conf

LFLAGS += -lglobal -lpantilt_interface -lparam_interface -lipc 

CFLAGS     +=  -g -O0 -ggdb 
CXXFLAGS   +=  -g -O0 -ggdb

MODULE_NAME = PANTILT
MODULE_COMMENT = Module for pantilt interface - direct perception

SOURCES =  pantilt_interface.c main.c io.c utils.c ipc.c pantilt-test.c

PUBLIC_INCLUDES = pantilt_interface.h pantilt_messages.h
PUBLIC_LIBRARIES = libpantilt_interface.a
PUBLIC_BINARIES = pantilt pantilt-test

TARGETS = libpantilt_interface.a pantilt pantilt-test

all: 

libpantilt_interface.a: pantilt_interface.o

pantilt:  io.o ipc.o utils.o main.o

pantilt-test:  pantilt-test.o 

include ../Makefile.rules


