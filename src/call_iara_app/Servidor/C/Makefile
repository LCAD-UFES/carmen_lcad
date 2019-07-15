
include ../../../Makefile.conf

# Module name and description
MODULE_NAME = CALL_IARA_SERVER
MODULE_COMMENT = Call IARA Server

LINK = g++

#LFLAGS += `pkg-config --libs opencv` -lparam_interface -lglobal -lbumblebee_basic_interface -lipc
#CFLAGS += -Wall `pkg-config --cflags opencv`
#CXXFLAGS += -Wall `pkg-config --cflags opencv` -std=c++11

SOURCES = call_iara_app_main.cpp 
PUBLIC_INCLUDES = call_iara_app_messages.h
PUBLIC_BINARIES = call_iara_main
TARGETS = call_iara_main

call_iara_main: call_iara_app_main.o

include ../../../Makefile.rules

