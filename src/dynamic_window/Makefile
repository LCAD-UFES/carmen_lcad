include ../Makefile.conf

# Module name and description
MODULE_NAME = Dynamic Window
MODULE_COMMENT = Dynamic Window Local Navigation Approach

LINK = g++

# Application specific include directories.
IFLAGS += 

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lglobal -lz -lipc -lm -lmap_interface -llocalize_ackerman_interface -lnavigator_ackerman_interface -lsimulator_ackerman_interface
CFLAGS += -Wall -O3 -std=c99
# Source code files (.c, .cpp) 
SOURCES = dynamic_window.c dynamic_window_main.c

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = dynamic_window

TARGETS = dynamic_window


dynamic_window:	dynamic_window_main.o dynamic_window.o 

include ../Makefile.rules
