
include ../../Makefile.conf

# Module name and description
MODULE_NAME = Publish Example
MODULE_COMMENT = Publish Example

LINK = g++
CXXFLAGS = -std=c++11

IFLAGS += -I/usr/include/python3.5

LFLAGS += -lglobal -lipc -lparam_interface

SOURCES = publish_example.c 
PUBLIC_INCLUDES = publish_example_messages.h

# Public libraries, linked to 'carmen/lib'.


PUBLIC_BINARIES = publish
TARGETS = publish

publish: publish_example.o

include ../../Makefile.rules

