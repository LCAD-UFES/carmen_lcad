include ../../Makefile.conf

CFLAGS += -D_REENTRANT
IFLAGS += 
LFLAGS += -lcanlib -pthread -lparam_interface -lbase_interface -lglobal -lipc -lm

MODULE_NAME = SEGWAY
MODULE_COMMENT = Segway RMP Base Module

ifeq (segway,$(findstring segway,$(ROBOT_TYPES)))
SOURCES = segwaycore.c segway_interface.c segway_test.c segway.c
PUBLIC_INCLUDES = segway_messages.h segway_interface.h
PUBLIC_LIBRARIES = libsegway_interface.a

TARGETS = libsegwaycore.a ../segway libsegway_interface.a segway_test
endif

# rules

libsegwaycore.a:	segwaycore.o

libsegway_interface.a:	segway_interface.o

../segway:		segway.o segway_ipc.o libsegwaycore.a

segway_test:		segway_test.o libsegway_interface.a

include ../../Makefile.rules
