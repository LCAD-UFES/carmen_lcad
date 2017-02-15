include ../Makefile.conf

MODULE_NAME = uDATMO
MODULE_COMMENT = Minimalist module for Detection And Tracking of Mobile Obstacles

CFLAGS += -Wall -std=c99
CXXFLAGS += -Wall -std=c++98
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
LINK = g++

LFLAGS += \
	-lparam_interface -lipc -lglobal \
	-L$(CARMEN_HOME)/sharedlib/libtf/src -ltf

SOURCES = \
	detector.cpp \
	sample_filter.cpp \
	udatmo.cpp

PUBLIC_INCLUDES = \
	udatmo.h

PUBLIC_LIBRARIES = \
	libudatmo.a

#PUBLIC_BINARIES = \
#	udatmo

TARGETS = \
	libudatmo.a

libudatmo.a: detector.o sample_filter.o udatmo.o
#udatmo: detector.o sample_filter.o ...

include ../Makefile.rules
