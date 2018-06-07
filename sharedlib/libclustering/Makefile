include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CXXFLAGS += -std=c++0x

MODULE_NAME = Clustering Library
MODULE_COMMENT = Clustering Library

SOURCES = dbscan.cpp
 
PUBLIC_INCLUDES = dbscan.h

PUBLIC_LIBRARIES = libdbscan.a

TARGETS = libdbscan.a

PUBLIC_LIBRARIES_SO =

libdbscan.a: dbscan.o

include $(CARMEN_HOME)/src/Makefile.rules