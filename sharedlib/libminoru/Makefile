include ../../src/Makefile.conf

MODULE_NAME = LIBMINORU
MODULE_COMMENT = MINORU Camera Lib

LINK = g++

CXXFLAGS += -O3 -fopenmp
LFLAGS += -fopenmp

CFLAGS += `pkg-config opencv --cflags`
LFLAGS += `pkg-config opencv --libs` 
 
SOURCES = libcam.cpp test_mono.cpp test_stereo.cpp
TARGETS = libminoru.a test_mono test_stereo

ifndef NO_PYTHON
TARGETS += libminoru.so.1
endif

test_mono: test_mono.o libminoru.a

test_stereo: test_stereo.o libminoru.a

libminoru.a: libcam.o

libminoru.so.1: libcam.o 

# rules
include ../../src/Makefile.rules

