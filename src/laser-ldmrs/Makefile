include ../Makefile.conf

# Module name and description
MODULE_NAME = SICK LD-MRS Package
MODULE_COMMENT = SICK LD-MRS Package

LINK = g++

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libsickldmrs/include
# Required default libraries to comunicate with Carmen Core.
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libsickldmrs 
LFLAGS += -lparam_interface -lipc -lglobal -llaser_ldmrs_interface -lsickldmrs -lpthread -lrt
CFLAGS += -Wall
# Source code files (.c, .cpp) 
SOURCES = laser_ldmrs_main.c laser_ldmrs_interface.c

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = laser_ldmrs_messages.h laser_ldmrs_interface.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = liblaser_ldmrs_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = laser_ldmrs

TARGETS = libsickldmrs liblaser_ldmrs_interface.a laser_ldmrs

PUBLIC_LIBRARIES_SO = liblaser_ldmrs_interface.so 

liblaser_ldmrs_interface.a: laser_ldmrs_interface.o

liblaser_ldmrs_interface.so.1: laser_ldmrs_interface.o

laser_ldmrs:	laser_ldmrs_main.o 

libsickldmrs:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libsickldmrs

clean: clean_libsickldmrs

clean_libsickldmrs:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libsickldmrs

# rules

include ../Makefile.rules

