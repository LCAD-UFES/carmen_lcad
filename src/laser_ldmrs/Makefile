include ../Makefile.conf

# Module name and description
MODULE_NAME = SICK LD-MRS Package
MODULE_COMMENT = SICK LD-MRS Package

LINK = g++

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libsickldmrs/include -I$(CARMEN_HOME)/sharedlib/libsickldmrs/src -I$(CARMEN_HOME)/sharedlib/libsickldmrs2/include 
# Required default libraries to comunicate with Carmen Core.
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libsickldmrs
LFLAGS += -lparam_interface -lipc -lglobal -llaser_ldmrs_interface -lsickldmrs -lpthread -lrt -lbase_ackerman_interface -lstereo_velodyne_interface
CXXFLAGS += -Wall -Wno-unused-variable
# Source code files (.c, .cpp) 
SOURCES = laser_ldmrs_main.cpp laser_ldmrs_interface.cpp laser_ldmrs_utils.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = laser_ldmrs_messages.h laser_ldmrs_interface.h laser_ldmrs_utils.h 

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = liblaser_ldmrs_interface.a liblaser_ldmrs_utils.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = laser_ldmrs

TARGETS = libsickldmrs liblaser_ldmrs_interface.a liblaser_ldmrs_utils.a laser_ldmrs

PUBLIC_LIBRARIES_SO = liblaser_ldmrs_interface.so 

liblaser_ldmrs_utils.a: laser_ldmrs_utils.o

liblaser_ldmrs_interface.a: laser_ldmrs_interface.o

liblaser_ldmrs_interface.so.1: laser_ldmrs_interface.o

laser_ldmrs: laser_ldmrs_main.o  liblaser_ldmrs_utils.a

libsickldmrs2:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libsickldmrs2
	
libsickldmrs:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libsickldmrs

clean: clean_libsickldmrs clean_libsickldmrs2

clean_libsickldmrs:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libsickldmrs
	
clean_libsickldmrs2:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libsickldmrs2

# rules

include ../Makefile.rules

