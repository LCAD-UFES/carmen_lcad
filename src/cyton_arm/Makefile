include ../Makefile.conf

MODULE_NAME = CYTON_ARM
MODULE_COMMENT = Cyton arm module.

LINK = g++
CXXFLAGS += 

CFLAGS += -DRENDER_DEBUG

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcyton/include/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcyton/lib/
LFLAGS += -lparam_interface -lipc -lglobal \
			-lcyton_arm_interface -Wl,-rpath,$(CARMEN_HOME)/sharedlib/libcyton/lib -lecFoundCore -lecCytonHardwareInterface -lecActinSERender


SOURCES = cyton_arm.cpp cyton_arm_core.cpp cyton_arm_interface.c cyton_arm_loop.c cyton_arm_listener.c

PUBLIC_INCLUDES = cyton_arm_messages.h cyton_arm_interface.h cyton_arm_core.h
PUBLIC_LIBRARIES = libcyton_arm_interface.a libcyton_arm_core.a

PUBLIC_BINARIES = cyton_arm

TARGETS = libcyton_arm_interface.a libcyton_arm_core.a cyton_arm cyton_arm_loop cyton_arm_listener

PUBLIC_LIBRARIES_SO = libcyton_arm_interface.so
ifndef NO_PYTHON
TARGETS += libcyton_arm_interface.so.1
endif

#ifndef NO_GRAPHICS
#IFLAGS += `$(GTK_CONFIG) --cflags` 
#LFLAGS += -lparam_interface -lipc -lglobal -lglobal_graphics -lbumblebee_basic_interface \
#	`$(GTK_CONFIG) --libs` 
#SOURCES += bumblebee_basic_view.c
#PUBLIC_BINARIES += bumblebee_basic_view
#TARGETS += bumblebee_basic_view
#endif


libcyton_arm_interface.a:		cyton_arm_interface.o

libcyton_arm_interface.so.1:	cyton_arm_interface.o

libcyton_arm_core.a:			cyton_arm_core.o

cyton_arm:						cyton_arm.o libcyton_arm_core.a libcyton_arm_interface.a

cyton_arm_loop:					cyton_arm_loop.o

cyton_arm_listener:				cyton_arm_listener.o

# rules

include ../Makefile.rules
