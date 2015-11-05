include ../Makefile.conf

# Module name and description
MODULE_NAME = BASELINE EXTENSION
MODULE_COMMENT = Baseline Extension Module

LINK = g++

CFLAGS += -Wall

IFLAGS += 
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libvelodyne
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libvelodyne
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -lglobal -lipc -lbumblebee_basic_interface -llocalize_ackerman_interface `pkg-config --libs opencv` -lstereo_interface -lvelodyne_interface  -lparam_interface 
SOURCES = baseline_extension_synchronize_main.cpp baseline_extension_publish_main.cpp 

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = baseline_extension_synchronize baseline_extension_publish
PUBLIC_LIBRARIES_SO = 

TARGETS = baseline_extension_synchronize baseline_extension_publish

baseline_extension_synchronize: baseline_extension_synchronize_main.o
baseline_extension_publish: baseline_extension_publish_main.o

include ../Makefile.rules
