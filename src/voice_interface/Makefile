include ../Makefile.conf

# Module name and description
MODULE_NAME = voice_interface_PACKAGE
MODULE_COMMENT = Allows voice interaction with IARA

LINK = g++
CFLAGS += -Wall
PYTHON3_CONFIG := /usr/bin/python3-config
PYTHON3_INCLUDES := $(shell $(PYTHON3_CONFIG) --includes 2>/dev/null)
PYTHON3_EMBED_LIBS := $(shell $(PYTHON3_CONFIG) --embed --libs 2>/dev/null || $(PYTHON3_CONFIG) --libs 2>/dev/null)
PORCUPINE_LIB := $(firstword $(wildcard ${HOME}/packages_carmen/Porcupine/lib/linux/x86_64/libpv_porcupine.a ${HOME}/packages_carmen/porcupine-1.6/lib/linux/x86_64/libpv_porcupine.a))

# Application specific include directories.
IFLAGS += $(PYTHON3_INCLUDES)
LFLAGS += -pthread

# python3-config --libs
# Required default libraries to comunicate with Carmen Core.
LFLAGS += $(PYTHON3_EMBED_LIBS) \
			-L${HOME}/packages_carmen/Porcupine/lib/linux/x86_64 \
			-L${HOME}/packages_carmen/porcupine-1.6/lib/linux/x86_64 \
			-lparam_interface -lipc -lglobal -lvoice_interface_interface -lpv_porcupine -lasound \
			-lnavigator_ackerman_interface -lcurl -ljsoncpp
			
# Source code files (.c, .cpp) 
SOURCES = voice_interface_interface.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = voice_interface_messages.h voice_interface_interface.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libvoice_interface_interface.a

TARGETS = libvoice_interface_interface.a
PUBLIC_LIBRARIES_SO = libvoice_interface_interface.so

ifneq ($(strip $(PORCUPINE_LIB)),)
SOURCES += voice_interface_main.cpp voice_interface.cpp voice_interface_runtime.cpp porcupine_keyword.cpp
PUBLIC_BINARIES = voice_interface
TARGETS += voice_interface
endif

libvoice_interface_interface.a: voice_interface_interface.o

voice_interface: voice_interface_main.o porcupine_keyword.o voice_interface_runtime.o voice_interface.o libvoice_interface_interface.a

# rules

include ../Makefile.rules
