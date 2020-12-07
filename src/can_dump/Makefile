include ../Makefile.conf

# Module name and description
MODULE_NAME = CAN_DUMP_PACKAGE
MODULE_COMMENT = Allows dumping from a can interface in a Raspberry Pi

LINK = g++

# Application specific include directories.
IFLAGS += 

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lipc -lglobal -lcan_dump_interface
CFLAGS += -Wall
# Source code files (.c, .cpp) 
SOURCES = can_dump_main.cpp can_view_main.cpp can_dump_interface.c 

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = can_dump_messages.h can_dump_interface.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libcan_dump_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = can_dump can_view

TARGETS = libcan_dump_interface.a can_dump can_view
PUBLIC_LIBRARIES_SO = libcan_dump_interface.so 

libcan_dump_interface.a: can_dump_interface.o

can_dump: can_dump_main.o libcan_dump_interface.a

can_view: can_view_main.o libcan_dump_interface.a

# rules

include ../Makefile.rules

