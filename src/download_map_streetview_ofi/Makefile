include ../Makefile.conf

# Migração Ubuntu 26.04: o módulo linkava contra o OpenCV mas nunca pedia os -I dele.
# No Ubuntu 20.04 os headers ficavam em /usr/include/opencv2 (path padrão); o pacote do
# 26.04 põe tudo em /usr/include/opencv4, que exige -I explícito.
IFLAGS += `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv`

CFLAGS += -g -Wall
LFLAGS += -lglobal -lipc -lparam_interface \
			-llocalize_ackerman_interface -lgps_xyz_interface \
			-lcurl `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv` \
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



