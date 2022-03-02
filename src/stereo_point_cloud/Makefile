include ../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += -lglobal -lipc -lparam_interface -lstereo_interface -lm `pkg-config --libs opencv` -pthread

MODULE_NAME = stereo_point_cloud
MODULE_COMMENT = This module transforms the disparity information in a point cloud.

SOURCES = stereo_point_cloud.c stereo_point_cloud_interface.c
PUBLIC_INCLUDES = stereo_point_cloud_messages.h stereo_point_cloud_interface.h

PUBLIC_LIBRARIES = libstereo_point_cloud_interface.a
PUBLIC_BINARIES = stereo_point_cloud
MAN_PAGES =

TARGETS = libstereo_point_cloud_interface.a stereo_point_cloud

PUBLIC_LIBRARIES_SO = libstereo_point_cloud_interface.so

ifndef NO_PYTHON
TARGETS += libstereo_point_cloud_interface.so.1
endif

# rules
libstereo_point_cloud_interface.a:	stereo_point_cloud_interface.o

libstereo_point_cloud_interface.so.1:	stereo_point_cloud_interface.o

stereo_point_cloud:	stereo_point_cloud.o stereo_point_cloud_interface.a

include ../Makefile.rules



