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

ifeq ("$(CARMEN_LOGGING)","ON")
CFLAGS += -DBOOST_LOG_DYN_LINK -DCARMEN_LOGGING_ON
CXXFLAGS += -DBOOST_LOG_DYN_LINK -DCARMEN_LOGGING_ON
LFLAGS += \
	-lboost_log_setup \
	-lboost_log \
	-lboost_thread \
	-lboost_system
endif

SOURCES = \
	detector.cpp \
	obstacle.cpp \
	udatmo.cpp \
	udatmo_interface.cpp \
	udatmo_logger.cpp \
	udatmo_main.cpp \
	primitives.cpp

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

libudatmo.a: detector.o obstacle.o udatmo.o udatmo_interface.o primitives.o
libudatmo_interface.a: udatmo_interface.o
udatmo: detector.o obstacle.o udatmo.o udatmo_interface.o udatmo_main.o primitives.o
udatmo_logger: udatmo_logger.o udatmo_interface.o

include ../Makefile.rules
