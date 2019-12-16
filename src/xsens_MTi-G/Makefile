include ../Makefile.conf

# Module name and description
MODULE_NAME = XSENS MTIG
MODULE_COMMENT = Driver for the xsens_MTiG.

LINK = g++

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcmt/ -I$(CARMEN_HOME)/src/xsens_MTi-G/c_library_v2

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcmt/ -L/usr/local/ipc-3.9.1/src/lib/Linux-4.14/
LFLAGS += -lparam_interface -lipc -lglobal  -lgps_xyz_interface -lrotation_geometry -lcmt -lrt -lncurses -lpthread -lxsens_interface
CFLAGS += -Wall

SUBDIRS += 

# Source code files (.c, .cpp) 
SOURCES = example_linux.cpp xsens_mtig_main.cpp xsens_mtig_interface.c subscribe_xsens_mtig.c navio2_mavlink_main.c

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = xsens_mtig_messages.h xsens_mtig_interface.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libxsens_mtig_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = xsens_mtig subscribe_xsens_mtig

TARGETS = libcmt.a libxsens_mtig_interface.a xsens_mtig subscribe_xsens_mtig example navio2_mavlink
PUBLIC_LIBRARIES_SO = libxsens_mtig_interface.so

ifndef NO_PYTHON
TARGETS += libxsens_mtig_interface.so.1
endif


# rules

libcmt.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libcmt

libxsens_mtig_interface.a: xsens_mtig_interface.o

libxsens_mtig_interface.so.1: xsens_mtig_interface.o

xsens_mtig: xsens_mtig_main.o xsens_mtig_interface.o

subscribe_xsens_mtig: subscribe_xsens_mtig.o -lxsens_mtig_interface

example: example_linux.o

navio2_mavlink: navio2_mavlink_main.o

clean: clean_xsens

clean_xsens:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libcmt

include ../Makefile.rules



