include ../Makefile.conf

# Module name and description
MODULE_NAME = LANE_DETECTION
MODULE_COMMENT = Lane Detection Module

LINK = g++

#IFLAGS += 
LFLAGS += -lparam_interface -lipc -lglobal -llocalize_ackerman_interface -lmap_server_interface
CFLAGS += -Wall 
SOURCES = lane_detection_main.cpp lane_detection_interface.c 

PUBLIC_INCLUDES = lane_detection_messages.h lane_detection_interface.h
PUBLIC_LIBRARIES = liblane_detection_interface.a
PUBLIC_BINARIES = lane_detection
TARGETS = lane_detection liblane_detection_interface.a

liblane_detection_interface.a: lane_detection_interface.o
lane_detection: lane_detection_main.o lane_detection_interface.o

# rules

include ../Makefile.rules

