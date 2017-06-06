include ../Makefile.conf

# Module name and description
MODULE_NAME = LANE_MAPPER
MODULE_COMMENT = Lane Mapper Module

LINK = g++

#IFLAGS += 
LFLAGS += -lparam_interface -lipc -lglobal -llocalize_ackerman_interface -lmap_server_interface
LFLAGS += `pkg-config --libs opencv`
CFLAGS += -Wall 
SOURCES = lane_mapper_main.cpp lane_mapper_interface.c 

PUBLIC_INCLUDES = lane_mapper_messages.h lane_mapper_interface.h spline.h
PUBLIC_LIBRARIES = liblane_mapper_interface.a
PUBLIC_BINARIES = lane_mapper lane_mapper_generate_gt
TARGETS = lane_mapper liblane_mapper_interface.a lane_mapper_generate_gt lane_mapper_segment

liblane_mapper_interface.a: lane_mapper_interface.o
lane_mapper_generate_gt: lane_mapper_generate_gt.o
lane_mapper: lane_mapper_main.o lane_mapper_interface.o
lane_mapper_segment: lane_mapper_segment.o

# rules

include ../Makefile.rules

