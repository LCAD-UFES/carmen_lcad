include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CXXFLAGS += -std=c++0x

MODULE_NAME = RL_PID
MODULE_COMMENT = Reinforciment Learning Based PID 

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol/pid
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcarmodel

LFLAGS += -lfann -lpid -lcarmodel  

SOURCES = rlpid.cpp
 
PUBLIC_INCLUDES = rlpid.h
#PUBLIC_LIBRARIES = libpid.a
#PUBLIC_BINARIES =

TARGETS = librlpid.a 
#PUBLIC_LIBRARIES_SO =

librlpid.a: rlpid.o

include $(CARMEN_HOME)/src/Makefile.rules