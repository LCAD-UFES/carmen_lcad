include ../Makefile.conf

# Module name and description
MODULE_NAME = ROAD_MAPPER
MODULE_COMMENT = Road Mapper Module

LINK = g++

IFLAGS += `pkg-config --cflags opencv` 
LFLAGS += -lparam_interface -lipc -lglobal -llocalize_ackerman_interface -lmap_server_interface -lgrid_mapping -lmap_io -lmap_interface
LFLAGS += `pkg-config --libs opencv` -lpng
CFLAGS += -Wall 
SOURCES = road_mapper_main.cpp road_mapper_interface.c 
SOURCES += road_mapper_image_splitter.cpp road_mapper_display_map2.cpp road_mapper_display_map3.cpp
SOURCES += road_mapper_generate_db.cpp

PUBLIC_INCLUDES = road_mapper_messages.h road_mapper_interface.h road_mapper.h
PUBLIC_LIBRARIES = libroad_mapper_interface.a libroad_mapper.a
PUBLIC_BINARIES = road_mapper
TARGETS = libroad_mapper_interface.a libroad_mapper.a road_mapper
TARGETS += road_mapper_image_splitter road_mapper_display_map2 road_mapper_display_map3 
TARGETS += road_mapper_generate_db

libroad_mapper_interface.a: road_mapper_interface.o
libroad_mapper.a: road_mapper.o
road_mapper: road_mapper_main.o road_mapper_interface.o
road_mapper_image_splitter: road_mapper_image_splitter.o
road_mapper_display_map2: road_mapper_display_map2.o
road_mapper_display_map3: road_mapper_display_map3.o road_mapper.o
road_mapper_generate_db: road_mapper_generate_db.o road_mapper.o

# rules

include ../Makefile.rules

