include ../../src/Makefile.conf

MODULE_NAME = LIBROBOSENSE
MODULE_COMMENT = ROBOSENSE Library for UDP Access

LINK = g++
CXXFLAGS += -g
LFLAGS += -lipc -lglobal

PUBLIC_LIBRARIES = librobosense.a

SOURCES = input.cpp rsdriver.cpp main.cpp
TARGETS = test librobosense.a 

ifndef NO_PYTHON
TARGETS += librobosense.so.1
endif

test: main.o librobosense.a

librobosense.a: input.o rsdriver.o

librobosense.so.1: input.o rsdriver.o

clean: clean_librobosense

clean_librobosense:
	rm -rf *.o *.a *~
				 
# rules
include ../../src/Makefile.rules
