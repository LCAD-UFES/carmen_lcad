
include ../../Makefile.conf

# Module name and description
MODULE_NAME = print_ipc_message 
MODULE_COMMENT = Modulo para printar mensagens do IPC

LINK ?= g++
CXXFLAGS += -std=c++11 -Wno-sign-compare -Wno-unused-parameter

IFLAGS += -I/usr/include/python3.5

LFLAGS += -lglobal -lipc -lparam_interface -lpthread

SOURCES = print_ipc_message_main.cpp
#PUBLIC_INCLUDES = publish_example_messages.h

# Public libraries, linked to 'carmen/lib'.


PUBLIC_BINARIES = print_ipc_message 
TARGETS = print_ipc_message 

print_ipc_message: print_ipc_message_main.o

include ../../Makefile.rules

