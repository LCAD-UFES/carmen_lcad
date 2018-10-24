include ../Makefile.conf

# Module name and description
MODULE_NAME = voice_interface_PACKAGE
MODULE_COMMENT = Allows voice interaction with IARA

LINK = g++

# Application specific include directories.
IFLAGS += -I /usr/include/python2.7

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lpython2.7 -lparam_interface -lipc -lglobal -lvoice_interface_interface
CFLAGS += -Wall
# Source code files (.c, .cpp) 
SOURCES = voice_interface_main.cpp voice_interface_interface.cpp voice_interface.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = voice_interface_messages.h voice_interface_interface.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libvoice_interface_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = voice_interface 

TARGETS = libvoice_interface_interface.a voice_interface 
PUBLIC_LIBRARIES_SO = libvoice_interface_interface.so 

libvoice_interface_interface.a: voice_interface_interface.o voice_interface.o

voice_interface: voice_interface_main.o voice_interface.o libvoice_interface_interface.a

# rules

include ../Makefile.rules

