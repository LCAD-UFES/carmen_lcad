include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CFLAGS += -std=c99

MODULE_NAME = RL_PID
MODULE_COMMENT = Reinforciment Learning Based PID 

#IFLAGS += 
#LFLAGS += 

SOURCES = rlpid.cpp
 
PUBLIC_INCLUDES = rlpid.h
#PUBLIC_LIBRARIES = libpid.a
#PUBLIC_BINARIES =

TARGETS = librlpid.a 
#PUBLIC_LIBRARIES_SO =

librlpid.a: rlpid.o

include $(CARMEN_HOME)/src/Makefile.rules