
include ../../../Makefile.conf

# Module name and description
MODULE_NAME = CALL_IARA_SERVER
MODULE_COMMENT = Call IARA Server

LINK = g++
CXXFLAGS = -std=c++11

IFLAGS += -I/usr/include/python3.5

LFLAGS += -lglobal -lipc -lparam_interface -lvoice_interface_interface `python3.5-config --libs` -lnavigator_ackerman_interface
#LFLAGS += `pkg-config --libs opencv` -lparam_interface -lglobal -lbumblebee_basic_interface -lipc
#CFLAGS += -Wall `pkg-config --cflags opencv`
#CXXFLAGS += -Wall `pkg-config --cflags opencv` -std=c++11

SOURCES = call_iara_app_main.c 
PUBLIC_INCLUDES = call_iara_app_messages.h

# Public libraries, linked to 'carmen/lib'.


PUBLIC_BINARIES = call_iara_main
TARGETS = call_iara_main

call_iara_main: call_iara_app_main.o

include ../../../Makefile.rules

