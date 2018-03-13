include ../Makefile.conf

MODULE_NAME = RDDF Graph
MODULE_COMMENT = Create and view RDDF Graphs

LINK = g++
CFLAGS += -Wall

#CXXFLAGS += -std=c++11
CXXFLAGS += -Wno-ignored-qualifiers

IFLAGS += \
	`pkg-config --cflags opencv` \

LFLAGS += \
	`pkg-config --libs opencv` \
	-lparam_interface \
	-lipc \
	-lglobal \
	-llocalize_ackerman_interface \
	-lmap_server_interface \
	-lgrid_mapping \
	-lmap_io \
	-lmap_interface

SOURCES = \
	rddf_graph_main.cpp \
	rddf_graph_utils.cpp \


PUBLIC_INCLUDES = \
	rddf_graph.h	

PUBLIC_BINARIES = \
	rddf_graph_main

TARGETS = rddf_graph_main

rddf_graph_main: rddf_graph_main.o rddf_graph.o rddf_graph_utils.o

include ../Makefile.rules
