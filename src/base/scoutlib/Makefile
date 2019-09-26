include ../../Makefile.conf

ifndef CYGWIN
CFLAGS += -fPIC 
endif
IFLAGS +=
LFLAGS += -lglobal -lipc -lparam_interface -lpthread

MODULE_NAME = "SCOUT"
MODULE_COMMENT = "Program for controlling Scout robot"

SOURCES = Ndirect.c scout_lib.c vel_test.c 
PUBLIC_LIBRARIES = libscout.a 
MAN_PAGES = 

TARGETS = libscout.a vel_test

vel_test:	vel_test.o Ndirect.o

libscout.a:  	scout_lib.o Ndirect.o 

include ../../Makefile.rules
