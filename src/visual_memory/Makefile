include ../Makefile.conf

# Module name and description
MODULE_NAME = VISUAL_MEMORY
MODULE_COMMENT = MAE Visual Memory Module

LINK = g++

# Application specific include directories.
IFLAGS += -I$(MAEHOME)/include
IFLAGS += `pkg-config --cflags opencv`

# Required default libraries to comunicate with Carmen Core.

LFLAGS += `pkg-config --libs opencv`
LFLAGS += -L$(MAEHOME)/lib -L/usr/local/lib -L/usr/X11R6/lib
LFLAGS += -lnet_conn -lforms -lGA -lX11 -lglut -lGLU -lGL -lm -lfl -lrt -lz -lparam_interface -lipc -lglobal -fopenmp
#LFLAGS += -lbumblebee_basic_interface -lstereo_interface
CFLAGS += -O4 -Wall

MAE_FILES = visual_memory.con
# Source code files (.c, .cpp) 
SOURCES = visual_memory.c visual_memory_user_functions/visual_memory_user_functions.c \
	visual_memory_user_functions/visual_memory_filters.c visual_memory_interface.c \
	visual_memory_user_functions/visual_memory_utils.c

PUBLIC_INCLUDES = visual_memory_interface.h visual_memory_messages.h
# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libvisual_memory_interface.a 

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = visual_memory

TARGETS = visual_memory libvisual_memory_interface.a

# If you set graphics support in configure (only for GTK).
#ifndef NO_GRAPHICS
#IFLAGS += `$(GTK_CONFIG) --cflags` 
#LFLAGS += -lglobal_graphics `$(GTK_CONFIG) --libs` 
#SOURCES += visual_memory_view.c
#PUBLIC_BINARIES += visual_memory_view
#TARGETS += visual_memory_view
#endif

libvisual_memory_interface.a: visual_memory_interface.o

visual_memory: visual_memory.o libvisual_memory_interface.a \
	visual_memory_user_functions/visual_memory_user_functions.o visual_memory_user_functions/visual_memory_filters.o visual_memory_user_functions/visual_memory_utils.o

#visual_memory_handlers.o

#visual_memory_view: visual_memory_view.o libvisual_memory_interface.a

clean:	clean_user_functions

clean_user_functions:
	rm -rf *.con~
	rm -rf *.core
	rm -rf *.cmlx~
	rm -rf *~
	rm -rf *.o
	$(MAKE) clean -C visual_memory_user_functions

# rules

include ../Makefile.rules
