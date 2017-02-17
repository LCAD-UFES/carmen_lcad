include ../../Makefile.conf
LINK = g++
CFLAGS +=  -o  
IFLAGS += `pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv`

MODULE_NAME = SPLIT LOG
MODULE_COMMENT = Utility to split logs

SOURCES = dataset_creator.cpp
PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = dataset_creator 

TARGETS = dataset_creator 

dataset_creator : dataset_creator.o

include ../../Makefile.rules
