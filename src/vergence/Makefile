include ../Makefile.conf

# Module name and description
MODULE_NAME = VERGENCE
MODULE_COMMENT = MAE Vergence Module

LINK = g++

# Application specific include directories.
IFLAGS += -I$(MAEHOME)/include
IFLAGS += `pkg-config --cflags opencv`

# Required default libraries to comunicate with Carmen Core.

LFLAGS += -L$(MAEHOME)/lib -L/usr/local/lib -L/usr/X11R6/lib
LFLAGS += -lnet_conn -lforms -lGA -lX11 -lglut -lGLU -lGL -lm -lfl -lrt -lz -lparam_interface -lipc -lglobal -fopenmp
LFLAGS += -lbumblebee_basic_interface -lstereo_interface
LFLAGS += `pkg-config --libs opencv`

MAE_FILES = vergence.con
# Source code files (.c, .cpp) 
SOURCES = vergence.c vergence_user_functions/vergence_user_functions.cpp \
	vergence_user_functions/vergence_filters.c vergence_interface.c \
	vergence_bumblebee.c 

PUBLIC_INCLUDES = vergence_interface.h vergence_messages.h
# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libvergence_interface.a 

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = vergence vergence_bumblebee 

TARGETS = vergence vergence_bumblebee libvergence_interface.a

# If you set graphics support in configure (only for GTK).
ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += -lglobal_graphics `$(GTK_CONFIG) --libs` 
SOURCES += vergence_view.c
PUBLIC_BINARIES += vergence_view 
TARGETS += vergence_view
endif

libvergence_interface.a: vergence_interface.o

vergence: vergence.o vergence_handlers.o libvergence_interface.a \
	vergence_user_functions/vergence_user_functions.o vergence_user_functions/vergence_filters.o 

vergence_view: vergence_view.o libvergence_interface.a

vergence_bumblebee: vergence_bumblebee.o

clean:	clean_user_functions

clean_user_functions:
	rm -rf *.con~
	rm -rf *.core
	rm -rf *.cml~
	$(MAKE) clean -C vergence_user_functions

# rules

include ../Makefile.rules
