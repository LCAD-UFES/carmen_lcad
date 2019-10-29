include ../../src/Makefile.conf

MODULE_NAME = LIBVELODYNE2
MODULE_COMMENT = Velodyne Library for UDP Access

LINK = g++
CXXFLAGS += -g
LFLAGS += -lipc -lglobal
 
SOURCES = input.cpp driver.cpp main.cpp
TARGETS = test libvelodyne.a 

ifndef NO_PYTHON
TARGETS += libvelodyne.so.1
endif

test: main.o libvelodyne.a

libvelodyne.a: input.o driver.o

libvelodyne.so.1: input.o driver.o

clean: clean_libvelodyne2

clean_libvelodyne2:
	rm -rf *.o *.a *~
				 
# rules
include ../../src/Makefile.rules
