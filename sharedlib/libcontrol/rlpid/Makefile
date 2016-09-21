include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CXXFLAGS += -std=c++0x

MODULE_NAME = RL_PID
MODULE_COMMENT = Reinforciment Learning Based PID 

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcarneuralmodel

LFLAGS += -lcarneuralmodel -lfann  

SOURCES = rlpid.cpp
 
PUBLIC_INCLUDES = rlpid.h
#PUBLIC_LIBRARIES = libpid.a
#PUBLIC_BINARIES =

TARGETS = librlpid.a 
#PUBLIC_LIBRARIES_SO =

librlpid.a: rlpid.o

include $(CARMEN_HOME)/src/Makefile.rules