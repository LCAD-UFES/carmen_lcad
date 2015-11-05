include ../../Makefile.conf

# Module name and description
MODULE_NAME = MONTE_CARLO_LOCALIZATION
MODULE_COMMENT = Monte Carlo Localization

LINK = g++

# Application specific include directories.
IFLAGS += 

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src 

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lipc -lglobal -lmonte_carlo_localization_interface -lvelodyne_interface -lbase_ackerman_interface -ltf \
			-lmap_server_interface -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_system -lboost_signals -lpthread
CFLAGS += -Wall

# Source code files (.c, .cpp) 
SOURCES = monte_carlo_localization_main.cpp ../../mapper/message_interpolation.cpp monte_carlo_localization_interface.c 

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = monte_carlo_localization_messages.h monte_carlo_localization_interface.h ../message_interpolation.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libmonte_carlo_localization_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = monte_carlo_localization 

TARGETS = libmonte_carlo_localization_interface.a monte_carlo_localization
PUBLIC_LIBRARIES_SO = libmonte_carlo_localization_interface.so 

libmonte_carlo_localization_interface.a: monte_carlo_localization_interface.o

libmonte_carlo_localization_interface.so.1: monte_carlo_localization_interface.o

monte_carlo_localization:	monte_carlo_localization_main.o 

# rules

include ../../Makefile.rules

