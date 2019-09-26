include ../Makefile.conf

MODULE_NAME = Road Network
MODULE_COMMENT = Create and view Road Networks

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
	-lmap_interface \
	-lGL \
	-lglut \
	-lGLU

SOURCES = \
	road_map_path_planning_main.cpp \
	road_map_path_planning_utils.cpp \


PUBLIC_INCLUDES = \
	road_map_path_planning_utils.h	

PUBLIC_BINARIES = \
	road_map_path_planning_main\
	

TARGETS = road_map_path_planning_main
		  

road_map_path_planning_main: road_map_path_planning_main.o road_map_path_planning_utils.o

include ../Makefile.rules