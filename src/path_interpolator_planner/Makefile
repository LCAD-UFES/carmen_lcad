include ../Makefile.conf

# Module name and description
MODULE_NAME = PATH_INTERPOLATOR
MODULE_COMMENT = Path Interpolator

LINK = g++

# Required default libraries to comunicate with Carmen Core.
IFLAGS += 
LFLAGS += -lparam_interface -lglobal -lipc -lbehavior_selector_interface -lmotion_planner_interface -lgsl -lgslcblas
CFLAGS += -Wall

SOURCES = path_interpolator_main.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = path_interpolator
TARGETS = path_interpolator

path_interpolator: path_interpolator_main.o

include ../Makefile.rules
