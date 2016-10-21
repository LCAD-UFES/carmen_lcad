include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
#CXXFLAGS += -std=c++0x -O4
CXXFLAGS += -std=c++0x

MODULE_NAME = MPC
MODULE_COMMENT = Model Predictive Controller 

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcarmodel
IFLAGS += -I$(CARMEN_HOME)/include/carmen

LFLAGS +=  -lfann -lgsl -lgslcblas -lcarmodel

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcarmodel 

SOURCES = mpc.cpp mpc_position_control.cpp
 
PUBLIC_INCLUDES = mpc.h
#PUBLIC_LIBRARIES = libpid.a
#PUBLIC_BINARIES =

TARGETS = libmpc.a
#PUBLIC_LIBRARIES_SO =

libmpc.a: mpc.o mpc_position_control.o

include $(CARMEN_HOME)/src/Makefile.rules
