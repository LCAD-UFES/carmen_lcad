include ../Makefile.conf

# Module name and description
MODULE_NAME = NEURAL_GLOBAL_LOCALIZER
MODULE_COMMENT = Image based Localization Module

LINK = g++

# Application specific include directories.
IFLAGS += -I$(MAEHOME)/include
IFLAGS += `pkg-config --cflags opencv`

# Required default libraries to comunicate with Carmen Core.

LFLAGS += -L$(MAEHOME)/lib -L/usr/local/lib -L/usr/X11R6/lib
LFLAGS += -lnet_conn -lforms -lGA -lX11 -lglut -lGLU -lGL -lm -lfl -lrt -lz -lparam_interface -lipc -lglobal -fopenmp
LFLAGS += -lbumblebee_basic_interface -lstereo_interface -lekf_odometry_interface
LFLAGS += `pkg-config --libs opencv`
CFLAGS += -O4 -Wno-unused-parameter 
CXXFLAGS += -O4 -Wno-unused-parameter

MAE_FILES = neural_global_localizer.con
# Source code files (.c, .cpp) 
SOURCES = neural_global_localizer.c neural_global_localizer_user_functions/neural_global_localizer_user_functions.cpp \
	neural_global_localizer_user_functions/neural_global_localizer_filter.cpp neural_global_localizer_interface.c

PUBLIC_INCLUDES = neural_global_localizer_interface.h neural_global_localizer_messages.h
# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libneural_global_localizer_interface.a 

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = neural_global_localizer 

TARGETS =  libneural_global_localizer_interface.a neural_global_localizer

# If you set graphics support in configure (only for GTK).
ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += -lglobal_graphics `$(GTK_CONFIG) --libs` 
endif

libneural_global_localizer_interface.a: neural_global_localizer_interface.o

neural_global_localizer: neural_global_localizer.o  libneural_global_localizer_interface.a \
						 neural_global_localizer_user_functions/neural_global_localizer_user_functions.o \
						 neural_global_localizer_user_functions/neural_global_localizer_filter.o

clean:	clean_user_functions

clean_user_functions:
	rm -rf *.con~
	rm -rf *.core
	rm -rf *.cml~
	$(MAKE) clean -C neural_global_localizer_user_functions

# rules

include ../Makefile.rules
