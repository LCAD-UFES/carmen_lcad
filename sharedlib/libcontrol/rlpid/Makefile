include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CFLAGS += -std=c99

MODULE_NAME = RLPID
MODULE_COMMENT = Reinforciment Learning Based PID 

#IFLAGS += 
#LFLAGS += 

SOURCES = rlpid.cpp
 
PUBLIC_INCLUDES = rlpid.h
#PUBLIC_LIBRARIES = libpid.a
#PUBLIC_BINARIES =

TARGETS = rlpid librlpid.a 
#PUBLIC_LIBRARIES_SO =

librlpid.a: rlpid.o

rlpid: rlpid.o

include $(CARMEN_HOME)/src/Makefile.rules