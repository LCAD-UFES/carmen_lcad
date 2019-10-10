
include ../../Makefile.conf

# Module name and description
MODULE_NAME = Subscribe Example
MODULE_COMMENT = Subscribe Example

LINK = g++
CXXFLAGS = -std=c++11

IFLAGS += -I/usr/include/python3.5

LFLAGS += -lglobal -lipc -lparam_interface

SOURCES = subscribe_example.c 
#PUBLIC_INCLUDES = publish_example_messages.h

# Public libraries, linked to 'carmen/lib'.


PUBLIC_BINARIES = subscribe
TARGETS = subscribe

subscribe: subscribe_example.o

include ../../Makefile.rules

