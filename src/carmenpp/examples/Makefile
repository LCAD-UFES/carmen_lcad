include ../../Makefile.conf

LINK=g++

LFLAGS +=  -lcarmen_carmenpp -lreadlog -lwritelog -lmap_interface -lglobal -lipc

CFLAGS += -O3
CXXFLAGS += -O3

MODULE_NAME = CARMENPPEXAMPLES
MODULE_COMMENT = Examples for the CPP Wrapper for Carmen

SOURCES =  read_logfile.cpp  test.cpp

PUBLIC_INCLUDES  = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES  = 

TARGETS =  read_logfile test

read_logfile: read_logfile.o

test: test.o

include ../../Makefile.rules
