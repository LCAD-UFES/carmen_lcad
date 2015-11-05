include ../Makefile.conf

# Module name and description
MODULE_NAME = NEURAL_LOCALIZER
MODULE_COMMENT = Neural Computer Localizer Module 

LINK = g++

CFLAGS += -Wno-unused-parameter

# Application specific include directories.
IFLAGS += -I$(MAEHOME)/include
IFLAGS += `pkg-config --cflags opencv`

# Required default libraries to comunicate with Carmen Core.
LFLAGS += `pkg-config --libs opencv`
LFLAGS += -L$(MAEHOME)/lib -L/usr/local/lib -L/usr/X11R6/lib
LFLAGS += -lnet_conn -lforms -lGA -lX11 -lglut -lGLU -lGL -lm -lfl -lrt -lz -fopenmp 	\
		  -lparam_interface -lipc -lglobal -lmap_interface -lnavigator_interface

MAE_FILES = neural_localizer.con
# Source code files (.c, .cpp) 
SOURCES = neural_localizer.c neural_localizer_user_functions/neural_localizer_user_functions.c \
		  neural_localizer_user_functions/neural_localizer_filters.c neural_localizer_user_functions/neural_localizer_utils.c \
		  neural_localizer_interface.c

PUBLIC_INCLUDES = neural_localizer_interface.h  neural_localizer_messages.h
# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libneural_localizer_interface.a 

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = neural_localizer


TARGETS = neural_localizer libneural_localizer_interface.a

# If you set graphics support in configure (only for GTK).
ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += -lglobal_graphics `$(GTK_CONFIG) --libs` 
SOURCES += 
PUBLIC_BINARIES += 
TARGETS += 
endif

libneural_localizer_interface.a: neural_localizer_interface.o

neural_localizer: neural_localizer.o neural_localizer_user_functions/neural_localizer_user_functions.o neural_localizer_user_functions/neural_localizer_filters.o neural_localizer_user_functions/neural_localizer_utils.o libneural_localizer_interface.a

clean:	clean_user_functions

clean_user_functions:
	rm -rf *.con~
	rm -rf *.core
	rm -rf *.cml~
	$(MAKE) clean -C neural_localizer_user_functions

# rules

include ../Makefile.rules

