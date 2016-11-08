include ../Makefile.conf

ifdef CARMEN_DEBUG
CFLAGS += -DDEBUG
endif

ifndef CARMEN_DEBUG
CFLAGS += -O3 -fopenmp -pg
endif


LINK = g++
CFLAGS += -Wall
IFLAGS += -I/usr/local/include
LFLAGS += -lglobal -lparam_interface -lipc -lGL -lGLU -lglut \
	  -lstereo_interface -lbumblebee_basic_interface \
	 `pkg-config --libs opencv`

MODULE_NAME = V_DISPARITY
MODULE_COMMENT = V-Disparity Library and Viewer

SOURCES = v_disparity.cpp
PUBLIC_INCLUDES = v_disparity.h
 
ifndef NO_CUDA
SOURCES += v_disparity_GPU.cu
endif

PUBLIC_LIBRARIES = libv_disparity.a
PUBLIC_BINARIES =
MAN_PAGES =

TARGETS = libv_disparity.a

PUBLIC_LIBRARIES_SO = 

ifndef NO_PYTHON
TARGETS += 
endif

ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += -lparam_interface -lipc -lglobal -lglobal_graphics \
	`$(GTK_CONFIG) --libs` 
SOURCES += v_disparity_view.c
PUBLIC_BINARIES += v_disparity_view
TARGETS += v_disparity_view
endif

ifndef NO_CUDA
libv_disparity.a: v_disparity.o  v_disparity_GPU.o
else
libv_disparity.a: v_disparity.o
endif

v_disparity_view: v_disparity_view.o libv_disparity.a

include ../Makefile.rules

