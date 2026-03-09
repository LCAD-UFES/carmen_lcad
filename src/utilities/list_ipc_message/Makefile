
include ../../Makefile.conf

#SUBDIRS = fake_print

# Module name and description
MODULE_NAME = list_ipc_message 
MODULE_COMMENT = Modulo para printar mensagens do IPC

LINK ?= g++
CXXFLAGS += -std=c++11 -Wno-deprecated-declarations -Wno-deprecated-copy 

IFLAGS += -I/usr/include/python3.5 $(shell pkg-config --cflags gtkmm-2.4) -I$(CARMEN_HOME)/src/ipc -I$(CARMEN_HOME)/src/ipc/comview

LFLAGS += -lglobal -lparam_interface -lpthread $(shell pkg-config --libs gtkmm-2.4) -lipc
		  
SOURCES = list_ipc_message_main.cpp list_ipc_message_fake_print.cpp
#PUBLIC_INCLUDES = publish_example_messages.h

# Public libraries, linked to 'carmen/lib'.


PUBLIC_BINARIES = list_ipc_message 
TARGETS = list_ipc_message 

list_ipc_message: list_ipc_message_main.o list_ipc_message_fake_print.o

include ../../Makefile.rules

