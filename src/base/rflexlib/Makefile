include ../../Makefile.conf

ifndef CYGWIN
CFLAGS += -fPIC
endif
IFLAGS +=
LFLAGS += -lglobal -lipc -lparam_interface

MODULE_NAME = "RFLEX"
MODULE_COMMENT = "Program for controlling a Rflex-based RWI robot \
By Dirk Haehnel and Nick Roy"

SOURCES = rflex_test.c rflex-io.c rflex_brake.c rflex_lib.c
PUBLIC_INCLUDES =  rflex_main.h
PUBLIC_LIBRARIES = librflex.a 
PUBLIC_BINARIES = rflex 
MAN_PAGES = 

TARGETS = librflex.a rflex_test rflex_brake 

rflex_test:	rflex_test.o

rflex_brake:	rflex_brake.o

librflex.a: rflex_lib.o rflex-io.o

rflex2: ../base/base_main.o librflex2.a 

include ../../Makefile.rules



