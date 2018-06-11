include ../Makefile.conf

MODULE_NAME = CAMERA_VIEWER
MODULE_COMMENT = Viewer for objects on camera image

LINK = g++

LFLAGS += -lcamera_interface -lparam_interface -lipc -lglobal -lbumblebee_basic_interface `pkg-config --libs opencv`

CFLAGS +=

SOURCES = camera_viewer_main.cpp

PUBLIC_INCLUDES =

PUBLIC_LIBRARIES =

PUBLIC_BINARIES = camera_viewer

TARGETS = camera_viewer
PUBLIC_LIBRARIES_SO =

camera_viewer:	camera_viewer_main.o

include ../Makefile.rules