include $(CARMEN_HOME)/src/Makefile.conf

# Module name and description
MODULE_NAME = DYNAMIC_OBJECT_DETECTOR
MODULE_COMMENT = A GridMap based Dynamic Object Detector

LINK = g++

# Application specific include directories.
IFLAGS += 

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lipc -lglobal -lmapper_interface -ldynamic_object_detector_interface -lmap_server_interface -lfused_odometry_interface -llocalize_ackerman_interface
CFLAGS += -Wall
# Source code files (.c, .cpp) 
SOURCES = dynamic_object_detector_main.cpp dynamic_object_detector_interface.c

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = dynamic_object_detector_messages.h dynamic_object_detector_interface.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libdynamic_object_detector_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = dynamic_object_detector

TARGETS = libdynamic_object_detector.a dynamic_object_detector
PUBLIC_LIBRARIES_SO = libdynamic_object_detector_interface.so

ifndef NO_PYTHON
TARGETS += libdynamic_object_detector_interface.so.1
endif

# If you set graphics support in configure (only for GTK).
ifndef NO_GRAPHICS
FLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += `pkg-config --libs gtk+-2.0`
#Source code file (.c)
TARGETS += 
SOURCES += 
endif

libdynamic_object_detector_interface.a:	dynamic_object_detector_interface.o

libdynamic_object_detector_interface.so.1:	dynamic_object_detector_interface.o

dynamic_object_detector:	dynamic_object_detector_main.o libdynamic_object_detector_interface.a 

# rules

include $(CARMEN_HOME)/src/Makefile.rules
