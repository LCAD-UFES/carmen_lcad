include ../Makefile.conf

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcontrol

LFLAGS += -lglobal -lipc -lparam_interface -llocalize_ackerman_interface \
	  -lrobot_ackerman_interface -lcontrol

MODULE_NAME = BASE ACKERMAN
MODULE_COMMENT = "Interface for Ackerman robots"

SOURCES = base_ackerman.c base_ackerman_simulation.c base_ackerman_interface.c

PUBLIC_INCLUDES = base_ackerman_messages.h base_ackerman_interface.h
PUBLIC_LIBRARIES = libbase_ackerman_interface.a 
PUBLIC_BINARIES = base_ackerman 
MAN_PAGES =

TARGETS = base_ackerman libbase_ackerman_interface.a


ifndef NO_PYTHON
PUBLIC_LIBRARIES_SO = libbase_ackerman_interface.so
TARGETS += libbase_ackerman_interface.so.1
endif


# rules

base_ackerman: base_ackerman.o base_ackerman_simulation.o

libbase_ackerman_interface.a: base_ackerman_interface.o

libbase_ackerman_interface.so.1: base_ackerman_interface.o

include ../Makefile.rules
