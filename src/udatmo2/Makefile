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
	-lmapper_interface \
	-lrddf_interface \
	-lparam_interface -lipc -lglobal \
	-L$(CARMEN_HOME)/sharedlib/libtf/src -ltf

# Add logging dependencies as required
ifeq ("$(CARMEN_LOGGING)","ON")
CXXFLAGS += -DBOOST_LOG_DYN_LINK -DCARMEN_LOGGING_ON
LFLAGS += \
	-lboost_log_setup \
	-lboost_log \
	-lboost_thread \
	-lboost_system
endif

# If you set graphics support in configure.
ifndef NO_GRAPHICS
#Carmen Global Graphic Library and Gtk+2 Graphics Libraries
LFLAGS += -lglobal_graphics
IFLAGS += `pkg-config --cflags gtk+-2.0 gmodule-export-2.0`
IFLAGS += `pkg-config --cflags gtkglext-1.0`
LFLAGS += `pkg-config --libs gtk+-2.0 gmodule-export-2.0`
LFLAGS += `pkg-config --libs gtkglext-1.0`
endif

SOURCES = \
	detector.cpp \
	obstacle.cpp \
	primitives.cpp \
	udatmo.cpp \
	udatmo_interface.cpp \
	udatmo_logger.cpp \
	udatmo_main.cpp

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

libudatmo.a: detector.o obstacle.o primitives.o udatmo.o udatmo_interface.o
libudatmo_interface.a: udatmo_interface.o
udatmo: detector.o obstacle.o primitives.o udatmo.o udatmo_interface.o udatmo_main.o
udatmo_logger: udatmo_logger.o udatmo_interface.o

include ../Makefile.rules
