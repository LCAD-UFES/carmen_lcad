include ../../src/Makefile.conf

MODULE_NAME = LIBVELODYNE2
MODULE_COMMENT = Velodyne Library for UDP Access using Variable Message

LINK = g++
CXXFLAGS += -g
LFLAGS += -lipc -lglobal
 
SOURCES = input.cpp driver2.cpp main.cpp
TARGETS = test libvelodyne2.a 

ifndef NO_PYTHON
TARGETS += libvelodyne2.so.1
endif

test: main.o libvelodyne2.a

libvelodyne2.a: input.o driver2.o

libvelodyne2.so.1: input.o driver2.o

clean: clean_libvelodyne2

clean_libvelodyne2:
	rm -rf *.o *.a *~
				 
# rules
include ../../src/Makefile.rules
