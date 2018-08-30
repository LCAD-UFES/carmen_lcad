include ../Makefile.conf

# Module name and description
MODULE_NAME = TRACKER
MODULE_COMMENT = MAE Tracker

LINK = g++

# Application specific include directories.
IFLAGS += `pkg-config --cflags opencv`
IFLAGS += -I$(MAEHOME)/include

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -L$(MAEHOME)/lib -L/usr/local/lib -L/usr/X11R6/lib 
LFLAGS += -lnet_conn -lforms -lGA -lX11 -lglut -lGLU -lGL -lm -lfl -lrt -lz -lparam_interface -lipc -lglobal -fopenmp
LFLAGS += -lbumblebee_basic_interface -lstereo_interface 
LFLAGS +=`pkg-config --libs opencv` -fopenmp
CFLAGS += -O4 -Wall -fopenmp
CXXFLAGS += -O4 -Wall -fopenmp

MAE_FILES = tracker.con
# Source code files (.c, .cpp) 
SOURCES = tracker.c tracker_user_functions/tracker_user_functions.cpp \
	tracker_user_functions/tracker_filters.c tracker_interface.c

PUBLIC_INCLUDES = tracker_interface.h tracker_messages.h
# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libtracker_interface.a 

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = tracker

TARGETS = tracker libtracker_interface.a

# If you set graphics support in configure (only for GTK).
ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += -lglobal_graphics `$(GTK_CONFIG) --libs` 
SOURCES += tracker_main.cpp
PUBLIC_BINARIES += tracker_main
TARGETS += tracker_main
endif

libtracker_interface.a: tracker_interface.o

tracker: tracker.o tracker_handlers.o libtracker_interface.a \
	tracker_user_functions/tracker_user_functions.o tracker_user_functions/tracker_filters.o 

tracker_main: tracker_main.o libtracker_interface.a

clean:	clean_user_functions

clean_user_functions:
	rm -rf *.con~
	rm -rf *.core
	rm -rf *.cml~
	$(MAKE) clean -C tracker_user_functions

# rules

include ../Makefile.rules
