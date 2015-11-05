include ../Makefile.conf

CFLAGS += -g -Wall
LFLAGS += -lglobal -lipc -lparam_interface \
			-llocalize_ackerman_interface -lgps_xyz_interface \
			-lcurl `pkg-config --libs opencv` \
			-lcarmen_gps_wrapper -lm -lgrid_mapping -lmap_io -lmap_interface

MODULE_NAME = DOWNLOAD_MAP_STREETVIEW
MODULE_COMMENT = This module download map images from google maps

SOURCES = download_map_streetview.c download_map_streetview_interface.c download_map_streetview_main.c
PUBLIC_INCLUDES = download_map_streetview_messages.h download_map_streetview_interface.h

PUBLIC_LIBRARIES = libdownload_map_streetview.a
PUBLIC_BINARIES = download_map_streetview
MAN_PAGES =

TARGETS = download_map_streetview libdownload_map_streetview.a

libdownload_map_streetview.a: download_map_streetview_interface.o
download_map_streetview: download_map_streetview.o download_map_streetview_main.o download_map_streetview_interface.o

include ../Makefile.rules



