include ../Makefile.conf

# Module name and description
MODULE_NAME = CAN_DUMP_PACKAGE
MODULE_COMMENT = Allows dumping from a can interface in a Raspberry Pi

LINK = g++
CFLAGS += -Wall

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/OpenJAUS/ojIARASim/include
LFLAGS += -L$(CARMEN_HOME)/sharedlib/OpenJAUS/ojIARASim/lib

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lojIARASim -lparam_interface -lglobal -lipc -pthread -lcan_dump_interface

# Source code files (.c, .cpp) 
SOURCES = can_dump_main.cpp can_view_main.cpp can_playback_main.cpp toyota_corolla_simulator_main.cpp can_dump_interface.c can_playback_one_can_main.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = can_dump_messages.h can_dump_interface.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libcan_dump_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = can_dump can_view can_playback toyota_corolla_simulator can_playback_one_can

TARGETS = OpenJAUS libcan_dump_interface.a can_dump can_view can_playback toyota_corolla_simulator can_playback_one_can
PUBLIC_LIBRARIES_SO = libcan_dump_interface.so 

OpenJAUS:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/OpenJAUS

libcan_dump_interface.a: can_dump_interface.o

can_dump: can_dump_main.o libcan_dump_interface.a

can_view: can_view_main.o libcan_dump_interface.a

can_playback: can_playback_main.o libcan_dump_interface.a

toyota_corolla_simulator: toyota_corolla_simulator_main.o

can_playback_one_can: can_playback_one_can_main.o libcan_dump_interface.a

# rules

include ../Makefile.rules

