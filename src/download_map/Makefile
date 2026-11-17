include ../Makefile.conf

# Migração Ubuntu 26.04: o módulo linkava contra o OpenCV mas nunca pedia os -I dele.
# No Ubuntu 20.04 os headers ficavam em /usr/include/opencv2 (path padrão); o pacote do
# 26.04 põe tudo em /usr/include/opencv4, que exige -I explícito.
IFLAGS += `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv`

CC = g++
LINK = g++
CFLAGS += -g -Wall
LFLAGS += -lglobal -lipc -lparam_interface \
			-llocalize_ackerman_interface -lgps_xyz_interface \
			-lcurl `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv` \
			-lcarmen_gps_wrapper -lm -lgrid_mapping -lmap_io -lmap_interface -pthread

MODULE_NAME = DOWNLOAD_MAP
MODULE_COMMENT = This module download map images from google maps

SOURCES = download_map.c download_map_interface.c download_map_main.c offiline_download_map.c
PUBLIC_INCLUDES = download_map_messages.h download_map_interface.h

PUBLIC_LIBRARIES = libdownload_map.a
PUBLIC_BINARIES = download_map
MAN_PAGES =

TARGETS = libdownload_map.a download_map offiline_download_map

libdownload_map.a: download_map_interface.o

download_map: download_map.o libdownload_map.a download_map_main.o

offiline_download_map: download_map.o offiline_download_map.o download_map_interface.o

include ../Makefile.rules



