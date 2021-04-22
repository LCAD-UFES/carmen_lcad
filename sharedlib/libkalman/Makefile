include ../../src/Makefile.conf

MODULE_NAME = Lib Kalman
MODULE_COMMENT = Kalman, Extended Kalman e IMM filters Implementations

LINK = g++
CFLAGS += -Wall -std=c++0x
CXXFLAGS += -Wall -std=c++0x

LFLAGS += \
	-lglobal -lipc -lviso

SOURCES = \
	kalman.cpp \
	kalman_test.cpp

PUBLIC_INCLUDES = kalman.h

PUBLIC_LIBRARIES = libkalman.a

PUBLIC_BINARIES = 

TARGETS = kalman_test 
	
libkalman.a: kalman.o

kalman_test: kalman_test.o libkalman.a

include ../../src/Makefile.rules
