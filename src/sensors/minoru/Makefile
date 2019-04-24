include $(CARMEN_HOME)/src/Makefile.conf

# Module name and description
MODULE_NAME = MINORU_PACKAGE
MODULE_COMMENT = Minoru Package 

LINK = g++

CXXFLAGS += -Wall -fopenmp

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -fopenmp -lparam_interface -lipc -lglobal 

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libelas
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libelas

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libminoru
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libminoru

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libminorucalib
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libminorucalib

IFLAGS += 
LFLAGS += -lminoru -lminorucalib

IFLAGS += `pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv`

# Source code files (.c, .cpp) 
SOURCES = minoru_main.cpp minoru_interface.c

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = minoru_messages.h minoru_interface.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libminoru_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = minoru

TARGETS = libminoru.a libminorucalib.a libminoru_interface.a minoru minorucalib
PUBLIC_LIBRARIES_SO = libminoru_interface.so 

ifndef NO_PYTHON
TARGETS += libminoru_interface.so.1
endif

ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += `$(GTK_CONFIG) --libs` -lglobal_graphics -lglib-2.0
SOURCES += minoru_view.c
PUBLIC_BINARIES += minoru_view
TARGETS += minoru_view
endif

libminoru_interface.a:	minoru_interface.o

libminoru_interface.so.1:	minoru_interface.o

minoru:	minoru_main.o libminoru_interface.a 

minoru_view: minoru_view.o libminoru_interface.a

libminoru.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libminoru 

libminorucalib.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libminorucalib 

minorucalib:
	$(MAKE) -C calibration

clean: clean_minoru

clean_minoru:
	$(MAKE) clean -C calibration

# rules

include $(CARMEN_HOME)/src/Makefile.rules
