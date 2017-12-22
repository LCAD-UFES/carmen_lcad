include ../Makefile.conf

MODULE_NAME = Virtual Scan
MODULE_COMMENT = Abstract polar 2D representation of 2D or 3D range scanner data

CC = cc -std=c++0x
CFLAGS += -Wall -std=c++0x
CXXFLAGS += -Wall -std=c++0x
LINK = g++

LFLAGS += \
	-lgrid_mapping -lmap_io -lmap_interface -lmapper_interface -lparam_interface -lipc -lglobal -lmap_server_interface -lmoving_objects_interface

SOURCES = \
	datmo_mcmc.cpp \
	hypothesis_viewer.cpp \
	line.cpp \
	neighborhood_graph.cpp \
	obstacle.cpp \
	parameters.cpp \
	point.cpp \
	posterior.cpp \
	random.cpp \
	reading.cpp \
	readings.cpp \
	rectangle.cpp \
	segment.cpp \
	segment_viewer.cpp \
	track.cpp \
	tracker.cpp \
	tracks.cpp \
	virtual_scan.cpp \
	virtual_scan_interface.cpp \
	virtual_scan_main.cpp \
	virtual_scan_neighborhood_graph.cpp

PUBLIC_INCLUDES = \
	virtual_scan_interface.h \
	virtual_scan_messages.h

PUBLIC_LIBRARIES = \
	libvirtual_scan_interface.a

PUBLIC_BINARIES = \
	virtual_scan

TARGETS = \
	datmo_mcmc \
	libvirtual_scan_interface.a \
	hypothesis_viewer \
	message_viewer \
	reading_viewer \
	segment_viewer \
	virtual_scan

# If you set graphics support in configure.
ifndef NO_GRAPHICS
#Carmen Global Graphic Library and Gtk+2 Graphics Libraries
IFLAGS += \
	`pkg-config --cflags gtk+-2.0 gmodule-export-2.0` \
	`pkg-config --cflags gtkglext-1.0`
LFLAGS += \
	-lglobal_graphics \
	`pkg-config --libs gtk+-2.0 gmodule-export-2.0` \
	`pkg-config --libs gtkglext-1.0`
endif

libvirtual_scan_interface.a: \
	virtual_scan_interface.o

datmo_mcmc: \
	datmo_mcmc.o \
	line.o \
	neighborhood_graph.o \
	obstacle.o \
	parameters.o \
	point.o \
	posterior.o \
	random.o \
	reading.o \
	readings.o \
	rectangle.o \
	segment.o \
	track.o \
	tracker.o \
	tracks.o

hypothesis_viewer: \
	hypothesis_viewer.o \
	neighborhood_graph.o \
	line.o \
	parameters.o \
	point.o \
	reading.o \
	segment.o

message_viewer: \
	message_viewer.o

reading_viewer: \
	point.o \
	reading.o \
	reading_viewer.o

segment_viewer: \
	line.o \
	parameters.o \
	point.o \
	reading.o \
	segment.o \
	segment_viewer.o

virtual_scan: \
	line.o \
	neighborhood_graph.o \
	obstacle.o \
	parameters.o \
	point.o \
	posterior.o \
	random.o \
	reading.o \
	readings.o \
	rectangle.o \
	segment.o \
	track.o \
	tracker.o \
	tracks.o \
	virtual_scan_main.o \
	virtual_scan.o \
	virtual_scan_interface.o \
	virtual_scan_neighborhood_graph.o

include ../Makefile.rules
