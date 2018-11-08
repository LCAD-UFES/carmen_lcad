include ../Makefile.conf

# Module name and description
MODULE_NAME = PARKING_ASSISTANT_PACKAGE
MODULE_COMMENT = Parking Assistant Package

LINK = g++

# Application specific include directories.
IFLAGS += 

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lglobal -lipc -lparking_assistant_interface \
		  -llocalize_ackerman_interface -lmapper_interface -lcollision_detection \
		  -lultrasonic_filter_interface -lbehavior_selector_interface \
		  -lnavigator_ackerman_interface -llocalize_ackerman_interface -lsimulator_ackerman_interface \
		  -lplayback_interface -lnavigator_gui2_interface -lGL -lglut -lGLU -lvoice_interface_interface -lespeak
CFLAGS += -Wall

#IFLAGS += `$(GTK_CONFIG) --cflags` 
#LFLAGS += `pkg-config --libs gtk+-2.0`
#LFLAGS += -lglobal_graphics -ljpeg

# Source code files (.c, .cpp) 
SOURCES = parking_assistant_main.cpp parking_assistant_interface.c parking_assistant_viewer_main.cpp message_interpolation.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = parking_assistant_messages.h parking_assistant_interface.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libparking_assistant_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = parking_assistant parking_assistant_viewer

TARGETS = libparking_assistant_interface.a parking_assistant parking_assistant_viewer
PUBLIC_LIBRARIES_SO = libparking_assistant_interface.so

ifndef NO_PYTHON
TARGETS += libparking_assistant_interface.so.1
endif

# If you set graphics support in configure (only for GTK).
ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += `pkg-config --libs gtk+-2.0`
LFLAGS += -lglobal_graphics -ljpeg
#Source code file (.c)
TARGETS +=
SOURCES += 
endif

libparking_assistant_interface.a:	parking_assistant_interface.o

libparking_assistant_interface.so.1:	parking_assistant_interface.o

parking_assistant:	parking_assistant_main.o libparking_assistant_interface.a

parking_assistant_viewer: parking_assistant_viewer_main.o

# rules

include ../Makefile.rules
