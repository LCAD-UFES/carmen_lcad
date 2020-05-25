include ../Makefile.conf

# Module name and description
MODULE_NAME = voice_interface_PACKAGE
MODULE_COMMENT = Allows voice interaction with IARA

LINK = g++
CFLAGS += -Wall -g

# Application specific include directories.
IFLAGS += -I/usr/include/python3.5 -I/usr/include/python3.6

# python3-config --libs
# Required default libraries to comunicate with Carmen Core.
LFLAGS += `python3.6-config --libs` -lparam_interface -lipc -lglobal -lvoice_interface_interface -lpv_porcupine -lasound \
			-lnavigator_ackerman_interface -lcurl -ljsoncpp
			
# Source code files (.c, .cpp) 
SOURCES = voice_interface_main.cpp voice_interface_interface.cpp voice_interface.cpp porcupine_keyword.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = voice_interface_messages.h voice_interface_interface.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libvoice_interface_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = voice_interface 

TARGETS = libvoice_interface_interface.a voice_interface 
PUBLIC_LIBRARIES_SO = libvoice_interface_interface.so

libvoice_interface_interface.a: voice_interface_interface.o voice_interface.o

voice_interface: voice_interface_main.o porcupine_keyword.o libvoice_interface_interface.a

# rules

include ../Makefile.rules

