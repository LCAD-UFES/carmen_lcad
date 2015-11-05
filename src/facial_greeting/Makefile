include ../Makefile.conf

# Module name and description
MODULE_NAME = FACIAL_GREETING
MODULE_COMMENT = User Friendly Facial Recognition

LINK = g++

# Application specific include directories.
IFLAGS += 

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lipc -lglobal -lbase_ackerman_interface -lespeak 
CFLAGS += -Wall
# Source code files (.c, .cpp) 
SOURCES = facial_greeting_main.cpp facial_greeting_interface.c

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = facial_greeting_messages.h facial_greeting_interface.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libfacial_greeting_interface.a		

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = facial_greeting

TARGETS = libfacial_greeting_interface.a facial_greeting 
PUBLIC_LIBRARIES_SO = 

ifndef NO_PYTHON
TARGETS += 
endif

# If you set graphics support in configure (only for GTK).
ifndef NO_GRAPHICS
FLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += `pkg-config --libs gtk+-2.0`
#Source code file (.c)
TARGETS += 
SOURCES += 
endif

# rules

libfacial_greeting_interface.a:		facial_greeting_interface.o

facial_greeting: facial_greeting_main.o libfacial_greeting_interface.a


include ../Makefile.rules
