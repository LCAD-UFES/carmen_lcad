include ../Makefile.conf

# Module name and description
MODULE_NAME = ROAD_MAPPER
MODULE_COMMENT = Road Mapper Module

LINK = g++

IFLAGS += `pkg-config --cflags opencv` 
LFLAGS += -lparam_interface -lipc -lglobal -llocalize_ackerman_interface -lmap_server_interface -lpng
LFLAGS += `pkg-config --libs opencv`
CFLAGS += -Wall 
SOURCES = road_mapper_main.cpp road_mapper_interface.c 
SOURCES += road_mapper_image_splitter.cpp road_mapper_display_map2.cpp road_mapper_display_map3.cpp

PUBLIC_INCLUDES = road_mapper_messages.h road_mapper_interface.h
PUBLIC_LIBRARIES = libroad_mapper_interface.a
PUBLIC_BINARIES = road_mapper
TARGETS = road_mapper libroad_mapper_interface.a
TARGETS += road_mapper_image_splitter road_mapper_display_map2 road_mapper_display_map3

libroad_mapper_interface.a: road_mapper_interface.o
road_mapper: road_mapper_main.o road_mapper_interface.o
road_mapper_image_splitter: road_mapper_image_splitter.o
road_mapper_display_map2: road_mapper_display_map2.o
road_mapper_display_map3: road_mapper_display_map3.o

# rules

include ../Makefile.rules

