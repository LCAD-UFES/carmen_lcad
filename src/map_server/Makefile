include ../Makefile.conf

# Module name and description
MODULE_NAME = MAP_SERVER
MODULE_COMMENT = 

LINK = g++

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models

IFLAGS += -I/usr/include/python3.5 -I/usr/include/python3.6 -I/usr/include/python3.8
IFLAGS += `pkg-config --cflags opencv` 

# Required default libraries to comunicate with Carmen Core.
LFLAGS +=  -lgrid_mapping -lprob_models -lz -lmap_io -lmap_util -lmap_interface -lglobal -lparam_interface -lnavigator_spline_interface \
	-lmapper_interface -llocalize_ackerman_interface  -lipc \
	-L/usr/lib64/libkml -lrrt_util_wrapper -lkmlbase -lkmldom -lkmlengine -lcarmen_gps_wrapper -llocalize_ackerman_core \
	-lrotation_geometry -ldownload_map -lrddf_interface -ltracker_interface -lnavigator_astar_interface \
	-lsimulator_ackerman_interface -lbehavior_selector_interface -lvoice_interface_interface

UBUNTU_VERSION ="$(shell cat /etc/issue | grep Ubuntu | cut -c8-12)"
#$(warning now we reached ifeq UBUNTU_VERSION=$(UBUNTU_VERSION))
ifeq ($(UBUNTU_VERSION),"16.04")
	LFLAGS += `python3.5-config --libs`
else ifeq ($(UBUNTU_VERSION),"18.04")
	LFLAGS += `python3.6-config --libs`
else
	LFLAGS += `python3.8-config --embed --libs` -pthread
endif

CFLAGS += -fopenmp -std=c99 
CXXFLAGS += -Wno-write-strings

LFLAGS += `pkg-config --libs opencv` 

# Source code files (.c, .cpp) 
SOURCES = map_server_main.cpp map_server_interface.c slice_map.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = map_server_interface.h map_server_messages.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libmap_server_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = map_server

TARGETS = libprob_models.a map_server libmap_server_interface.a slice_map

map_server: map_server_main.o libmap_server_interface.a

slice_map: slice_map.o 

libmap_server_interface.a: map_server_interface.o

# rules

libprob_models.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/prob_models/

clean: clean_prob_models

clean_prob_models:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/prob_models/
	
include ../Makefile.rules
