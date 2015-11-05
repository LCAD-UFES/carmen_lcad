include ../../src/Makefile.conf

CFLAGS +=
LFLAGS += -lGLEW -lGL -lGLU -lglut -lX11

MODULE_NAME = gl_plotter
MODULE_COMMENT = This library creates a window and plots data.

SOURCES = gl_plotter.c Window.c

TARGETS = libgl_plotter.a

ifdef CARMEN_DEBUG
CFLAGS += -DDEBUG
endif

libgl_plotter.a: gl_plotter.o Window.o

include ../../src/Makefile.rules



