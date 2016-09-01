include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CFLAGS += -std=c99

MODULE_NAME = PID
MODULE_COMMENT = Proportional Integral Derivative Controller 

#IFLAGS += 
#LFLAGS += 

SOURCES = pid.cpp
 
PUBLIC_INCLUDES = pid.h
#PUBLIC_LIBRARIES = libpid.a 
#PUBLIC_BINARIES = 

TARGETS = libpid.a  
#PUBLIC_LIBRARIES_SO =

libpid.a: pid.o

include $(CARMEN_HOME)/src/Makefile.rules
