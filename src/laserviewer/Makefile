include ../Makefile.conf

MODULE_NAME = "LASER VIEWER"
MODULE_COMMENT = "The viewer for SICK lasers"

SOURCES = laserviewer_main.cpp

IFLAGS +=  
LFLAGS += -lglobal -lipc -lGL -lglut -lGLU -llaser_interface -lparam_interface

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = laserviewer 

TARGETS = laserviewer

laserviewer: laserviewer_main.o

include ../Makefile.rules
