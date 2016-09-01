include ../../Makefile.conf

MODULE_NAME = tracker_opentld/3rdparty
MODULE_COMMENT = tracker_opentld/3rdparty

LINK = g++

CXXFLAGS += 
CFLAGS += 
IFLAGS += `pkg-config --cflags opencv` -I cvblobs  
LFLAGS += `pkg-config --libs opencv`
SUBDIRS += 

SOURCES = cvblobs/BlobContour.cpp \
	cvblobs/blob.cpp \
	cvblobs/ComponentLabeling.cpp \
	cvblobs/BlobProperties.cpp \
	cvblobs/BlobResult.cpp \
	cvblobs/BlobOperators.cpp \
	libconfig/scanner.c \
	libconfig/libconfig.c \
	libconfig/grammar.c \
	libconfig/libconfigcpp.c++ \
	libconfig/libconfigcpp.cc \
	libconfig/strbuf.c \
	libconfig/scanctx.c 

    
PUBLIC_INCLUDES = 
    
PUBLIC_LIBRARIES = lib3rdparty.a
TARGETS = lib3rdparty.a
PUBLIC_BINARIES = 

lib3rdparty.a: cvblobs/BlobContour.o \
	cvblobs/blob.o \
	cvblobs/ComponentLabeling.o \
	cvblobs/BlobProperties.o \
	cvblobs/BlobResult.o \
	cvblobs/BlobOperators.o \
	libconfig/scanner.o \
	libconfig/libconfig.o \
	libconfig/grammar.o \
	libconfig/libconfigcpp.o \
	libconfig/libconfigcpp.o \
	libconfig/strbuf.o \
	libconfig/scanctx.o 

include ../../Makefile.rules
