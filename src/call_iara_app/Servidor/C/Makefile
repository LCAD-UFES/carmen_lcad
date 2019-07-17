
include ../../../Makefile.conf

# Module name and description
MODULE_NAME = CALL_IARA_SERVER
MODULE_COMMENT = Call IARA Server

LINK = g++
CFLAGS += -Wall -g
IFLAGS += -I/usr/include/python3.5
LFLAGS += `python3.5-config --libs` -lparam_interface -lipc -lglobal -lvoice_interface_interface -lnavigator_ackerman_interface -lasound -lcurl -ljsoncpp
#LFLAGS += `pkg-config --libs opencv` -lparam_interface -lglobal -lbumblebee_basic_interface -lipc
#CFLAGS += -Wall `pkg-config --cflags opencv`
#CXXFLAGS += -Wall `pkg-config --cflags opencv` -std=c++11

SOURCES = call_iara_app_main.c 
PUBLIC_INCLUDES = call_iara_app_messages.h voice_interface_interface.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libvoice_interface_interface.a

PUBLIC_BINARIES = call_iara_main
TARGETS = call_iara_main

call_iara_main: call_iara_app_main.o

include ../../../Makefile.rules

