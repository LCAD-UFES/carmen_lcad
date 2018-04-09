include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CXXFLAGS += -std=c++0x

MODULE_NAME = CLUSTERING LIBRARY
MODULE_COMMENT = Clustering Library

#IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol

SOURCES = dbscan.cpp
 
PUBLIC_INCLUDES = dbscan.h
PUBLIC_LIBRARIES = libdbscan.a
#PUBLIC_BINARIES =

TARGETS = libdbscan.a  
PUBLIC_LIBRARIES_SO =

libdbscan.a: dbscan.o

include $(CARMEN_HOME)/src/Makefile.rules