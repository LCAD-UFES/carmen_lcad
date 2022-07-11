include ../Makefile.conf

# Module name and description
MODULE_NAME = VISUAL_SEARCH
MODULE_COMMENT = MAE Visual Search Module

LINK = g++

# Application specific include directories.
IFLAGS += -I$(MAEHOME)/include
IFLAGS += `pkg-config --cflags opencv`

# Required default libraries to comunicate with Carmen Core.

LFLAGS += `pkg-config --libs opencv`
LFLAGS += -L$(MAEHOME)/lib -L/usr/local/lib -L/usr/X11R6/lib
LFLAGS += -lnet_conn -lforms -lGA -lX11 -lglut -lGLU -lGL -lm -lfl -lrt -lz -fopenmp \
	-lparam_interface -lipc -lglobal -lbumblebee_basic_interface -lkinect_interface

CFLAGS += -Wall -DCML_SCRIPT
#CFLAGS += -g -Wall 

MAE_FILES = visual_search.con
# Source code files (.c, .cpp) 
SOURCES = visual_search.c visual_search_user_functions/visual_search_user_functions.c visual_search_user_functions/visual_search_filters.c visual_search_interface.c visual_search_bumblebee.c

PUBLIC_INCLUDES = visual_search_interface.h visual_search_messages.h
# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libvisual_search_interface.a 

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = visual_search visual_search_bumblebee


TARGETS = visual_search libvisual_search_interface.a

# If you set graphics support in configure (only for GTK).
ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += -lglobal_graphics `$(GTK_CONFIG) --libs` 
SOURCES += visual_search_view.c
PUBLIC_BINARIES += visual_search_view
TARGETS += visual_search_view visual_search_bumblebee
endif

libvisual_search_interface.a: visual_search_interface.o

visual_search: visual_search.o visual_search_user_functions/visual_search_user_functions.o visual_search_user_functions/visual_search_filters.o libvisual_search_interface.a

visual_search_view:	visual_search_view.o libvisual_search_interface.a

visual_search_bumblebee: visual_search_bumblebee.o

clean:	clean_user_functions

clean_user_functions:
	rm -rf *.con~
	rm -rf *.core
	rm -rf *.cml~
	$(MAKE) clean -C visual_search_user_functions

# rules

include ../Makefile.rules
