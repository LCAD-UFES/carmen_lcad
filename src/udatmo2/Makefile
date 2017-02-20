include ../Makefile.conf

MODULE_NAME = uDATMO
MODULE_COMMENT = Minimalist module for Detection And Tracking of Mobile Obstacles

CFLAGS += -Wall -std=c99
CXXFLAGS += -Wall -std=c++98
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
LINK = g++

LFLAGS += \
	-lcollision_detection \
	-llocalize_ackerman_interface \
	-lobstacle_distance_mapper_interface \
	-lrddf_interface \
	-lparam_interface -lipc -lglobal \
	-L$(CARMEN_HOME)/sharedlib/libtf/src -ltf

SOURCES = \
	detector.cpp \
	obstacles.cpp \
	udatmo.cpp \
	udatmo_interface.cpp \
	udatmo_logger.cpp \
	udatmo_main.cpp \
	udatmo_memory.c

PUBLIC_INCLUDES = \
	udatmo.h \
	udatmo_interface.h \
	udatmo_messages.h

PUBLIC_LIBRARIES = \
	libudatmo.a \
	libudatmo_interface.a

PUBLIC_BINARIES = \
	udatmo \
	udatmo_logger

TARGETS = \
	libudatmo.a \
	libudatmo_interface.a \
	udatmo \
	udatmo_logger

libudatmo.a: detector.o obstacles.o udatmo.o udatmo_interface.o udatmo_memory.o
libudatmo_interface.a: udatmo_interface.o
udatmo: detector.o obstacles.o udatmo.o udatmo_interface.o udatmo_main.o udatmo_memory.o
udatmo_logger: udatmo_logger.o udatmo_interface.o

include ../Makefile.rules
