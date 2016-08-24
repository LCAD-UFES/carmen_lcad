include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CFLAGS += -std=c99

MODULE_NAME = MPC
MODULE_COMMENT = Model Predictive Controller 

#IFLAGS += 
#LFLAGS += 

SOURCES = mpc.c
 
PUBLIC_INCLUDES = mpc.h
#PUBLIC_LIBRARIES = libpid.a
#PUBLIC_BINARIES =

TARGETS = libmpc.a  
#PUBLIC_LIBRARIES_SO =

libmpc.a: mpc.o

include $(CARMEN_HOME)/src/Makefile.rules