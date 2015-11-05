include ../../src/Makefile.conf

MODULE_NAME = LIB ALGLIB
MODULE_COMMENT = ALGLIB

LINK = g++
CXXFLAGS += -w -g -O3 
IFLAGS += -Isrc/

SOURCES = src/alglibinternal.cpp  src/fasttransforms.cpp  src/solvers.cpp \
		  src/alglibmisc.cpp      src/integration.cpp     src/specialfunctions.cpp \
		  src/ap.cpp              src/interpolation.cpp   src/statistics.cpp \
		  src/dataanalysis.cpp    src/linalg.cpp		  src/diffequations.cpp \
		  src/optimization.cpp	  tests/test_c.cpp
		
TARGETS = libalglib.a test

ifndef NO_PYTHON
TARGETS += libalglib.so.1
endif

test: tests/test_c.o libalglib.a

libalglib.a: src/alglibinternal.o  src/fasttransforms.o  src/solvers.o \
		  src/alglibmisc.o      src/integration.o     src/specialfunctions.o \
		  src/ap.o              src/interpolation.o   src/statistics.o \
		  src/dataanalysis.o    src/linalg.o		  src/diffequations.o \
		  src/optimization.o

libalglib.so.1: src/alglibinternal.o  src/fasttransforms.o  src/solvers.o \
		  src/alglibmisc.o      src/integration.o     src/specialfunctions.o \
		  src/ap.o              src/interpolation.o   src/statistics.o \
		  src/dataanalysis.o    src/linalg.o		  src/diffequations.o \
		  src/optimization.o

clean: clean_alglib

clean_alglib:
	rm -rf src/*.o tests/*.o *.o *.a *~
				 
# rules
include ../../src/Makefile.rules


