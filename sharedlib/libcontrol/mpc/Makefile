include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CXXFLAGS += -std=c++0x

MODULE_NAME = MPC
MODULE_COMMENT = Model Predictive Controller 

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcarneuralmodel
IFLAGS += -I$(CARMEN_HOME)/include/carmen

LFLAGS +=  -lfann -lgsl -lgslcblas -lcarneuralmodel

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcarneuralmodel 

SOURCES = mpc.cpp
 
PUBLIC_INCLUDES = mpc.h
#PUBLIC_LIBRARIES = libpid.a
#PUBLIC_BINARIES =

TARGETS = libmpc.a
#PUBLIC_LIBRARIES_SO =

libmpc.a: mpc.o

include $(CARMEN_HOME)/src/Makefile.rules
