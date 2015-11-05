include ../Makefile.conf

# Module name and description
MODULE_NAME = VISUAL_SEARCH_THIN
MODULE_COMMENT = MAE Visual Search Thin Module

LINK = g++

# Application specific include directories.
IFLAGS += `pkg-config --cflags opencv`
IFLAGS += -I$(MAEHOME)/include

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -L$(MAEHOME)/lib -L/usr/local/lib -L/usr/X11R6/lib
LFLAGS += -lnet_conn -lforms -lGA -lX11 -lglut -lGLU -lGL -lm -lfl -lrt -lz -lparam_interface -lipc -lglobal -fopenmp
LFLAGS += -lbumblebee_basic_interface -lneural_global_localizer_interface -lstereo_interface
LFLAGS +=`pkg-config --libs opencv` -fopenmp
CFLAGS += -O4 -Wall -fopenmp
CXXFLAGS += -O4 -Wall -fopenmp

MAE_FILES = visual_search_thin.con
# Source code files (.c, .cpp) 
SOURCES = visual_search_thin.c visual_search_thin_user_functions/visual_search_thin_user_functions.cpp \
	visual_search_thin_user_functions/visual_search_thin_filters.c visual_search_thin_interface.c

PUBLIC_INCLUDES = visual_search_thin_interface.h visual_search_thin_messages.h
# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libvisual_search_thin_interface.a 

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = visual_search_thin 

TARGETS = visual_search_thin libvisual_search_thin_interface.a

# If you set graphics support in configure (only for GTK).
ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += -lglobal_graphics `$(GTK_CONFIG) --libs` 
SOURCES += visual_search_thin_view.c
PUBLIC_BINARIES += visual_search_thin_view
TARGETS += visual_search_thin_view
endif

libvisual_search_thin_interface.a: visual_search_thin_interface.o

visual_search_thin: visual_search_thin.o visual_search_thin_handlers.o libvisual_search_thin_interface.a \
	visual_search_thin_user_functions/visual_search_thin_user_functions.o visual_search_thin_user_functions/visual_search_thin_filters.o 

visual_search_thin_view: visual_search_thin_view.o libvisual_search_thin_interface.a

clean:	clean_user_functions

clean_user_functions:
	rm -rf *.con~
	rm -rf *.core
	rm -rf *.cml~
	$(MAKE) clean -C visual_search_thin_user_functions

# rules

include ../Makefile.rules
