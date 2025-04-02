include ../../src/Makefile.conf

MODULE_NAME = LIBELAS
MODULE_COMMENT = Disparity Map

LINK = g++
CXXFLAGS += -w # -msse3 -O3 -fopenmp
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libviso2.3/src
LFLAGS += -lipc -lglobal -fopenmp -lm
 
SOURCES = descriptor.cpp elas.cpp filter.cpp main.cpp matrix.cpp triangle.cpp
TARGETS = libelas.a elas

ifndef NO_PYTHON
TARGETS += libelas.so.1
endif

elas: main.o libelas.a

libelas.a: descriptor.o elas.o filter.o matrix.o triangle.o

libelas.so.1: descriptor.o elas.o filter.o matrix.o triangle.o

clean: clean_libelas

clean_libelas:
	rm -rf *.o *.a *~
				 
# rules
include ../../src/Makefile.rules


