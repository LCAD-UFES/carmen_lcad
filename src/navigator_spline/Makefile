include ../Makefile.conf

# Module name and description
MODULE_NAME = NAVIGATOR SPLINE 
MODULE_COMMENT = navigator spline

LINK = g++
CXXFLAGS += -std=c++0x #-pg 

# Application specific include directories.
IFLAGS += 

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lipc -lglobal -lnavigator_spline_interface -lrddf_interface -ltracker_interface -llocalize_ackerman_interface  -lbehavior_selector_interface
CFLAGS += -Wall
# Source code files (.c, .cpp) 
SOURCES = navigator_spline_main.cpp navigator_spline_interface.c

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = navigator_spline_messages.h navigator_spline_interface.h skeleton_module_panel.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libnavigator_spline_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = navigator_spline

TARGETS = libnavigator_spline_interface.a navigator_spline
PUBLIC_LIBRARIES_SO = libnavigator_spline_interface.so

libnavigator_spline_interface.a: navigator_spline_interface.o

libnavigator_spline_interface.so.1: navigator_spline_interface.o

navigator_spline:	navigator_spline_main.o 

# rules

include ../Makefile.rules

