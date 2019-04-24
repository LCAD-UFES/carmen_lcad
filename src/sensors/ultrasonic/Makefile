include $(CARMEN_HOME)/src/Makefile.conf

# Module name and description
MODULE_NAME = ULTRASONIC_SENSOR
MODULE_COMMENT = Ultrasonic Sensor

LINK = g++

# Application specific include directories.
IFLAGS += 

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lglobal -lipc -lultrasonic_filter_interface -lGL -lglut -lGLU 
CFLAGS += -Wall
# Source code files (.c, .cpp) 
SOURCES = ultrasonic_filter_main.c ultrasonic_filter_interface.c ultrasonic_viewer_main.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = ultrasonic_filter_messages.h ultrasonic_filter_interface.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libultrasonic_filter_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = ultrasonic ultrasonic_viewer

TARGETS = libultrasonic_filter_interface.a ultrasonic ultrasonic_viewer
PUBLIC_LIBRARIES_SO = libultrasonic_filter_interface.so

ifndef NO_PYTHON
TARGETS += libultrasonic_filter_interface.so.1
endif

# If you set graphics support in configure (only for GTK).
ifndef NO_GRAPHICS
FLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += `pkg-config --libs gtk+-2.0`
#Source code file (.c)
TARGETS +=
#skeleton_gui
SOURCES += 
endif

libultrasonic_filter_interface.a:	ultrasonic_filter_interface.o

libultrasonic_filter_interface.so.1:	ultrasonic_filter_interface.o

ultrasonic:	ultrasonic_filter_main.o libultrasonic_filter_interface.a 

ultrasonic_viewer: ultrasonic_viewer_main.o

# rules
include $(CARMEN_HOME)/src/Makefile.rules
