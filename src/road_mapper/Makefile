include ../Makefile.conf

# Module name and description
MODULE_NAME = ROAD_MAPPER
MODULE_COMMENT = Road Mapper Module

LINK = g++

#IFLAGS += 
LFLAGS += -lparam_interface -lipc -lglobal -llocalize_ackerman_interface -lmap_server_interface
LFLAGS += `pkg-config --libs opencv`
CFLAGS += -Wall 
SOURCES = road_mapper_main.cpp road_mapper_interface.c 

PUBLIC_INCLUDES = road_mapper_messages.h road_mapper_interface.h spline.h
PUBLIC_LIBRARIES = libroad_mapper_interface.a
PUBLIC_BINARIES = road_mapper road_mapper_generate_gt
TARGETS = road_mapper libroad_mapper_interface.a road_mapper_generate_gt road_mapper_segment

libroad_mapper_interface.a: road_mapper_interface.o
road_mapper_generate_gt: road_mapper_generate_gt.o
road_mapper: road_mapper_main.o road_mapper_interface.o
road_mapper_segment: road_mapper_segment.o

# rules

include ../Makefile.rules

