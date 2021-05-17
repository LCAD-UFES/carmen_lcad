include ../../../src/Makefile.conf

MODULE_NAME = LIBVISO2.3
MODULE_COMMENT = Visual Odometry Library

LINK = g++
CXXFLAGS += -msse3 -Wno-unused-parameter -Wno-unused-but-set-variable

CFLAGS += -std=c11 -Wno-strict-overflow
CXXFLAGS += -std=c++11 -Wno-strict-overflow

LFLAGS += -lipc -lglobal
 
SOURCES = matrix.cpp triangle.cpp filter.cpp matcher.cpp reconstruction.cpp viso.cpp viso_mono.cpp viso_stereo.cpp

PUBLIC_INCLUDES = matrix.h

PUBLIC_LIBRARIES = libviso.a

TARGETS = libviso.a

ifndef NO_PYTHON
TARGETS += libviso.so.1
endif

libviso.a: matrix.o triangle.o filter.o matcher.o reconstruction.o viso.o viso_mono.o viso_stereo.o

libviso.so.1: matrix.o triangle.o filter.o matcher.o reconstruction.o viso.o viso_mono.o viso_stereo.o

clean: clean_libviso

clean_libviso:
	rm -rf *.o *.a *~
				 
# rules
include ../../../src/Makefile.rules

