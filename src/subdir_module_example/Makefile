include ../Makefile.conf

CFLAGS += 
IFLAGS += 
LFLAGS += -lstdc++ -lsubdir_1 -lsubdir_2
SUBDIRS += subdir_1 subdir_2

MODULE_NAME = subdir_module_example
MODULE_COMMENT = Module to show how to use subdirs

SOURCES = main.cpp
PUBLIC_BINARIES = teste_subdir
TARGETS = teste_subdir

teste_subdir: main.o

include ../Makefile.rules
