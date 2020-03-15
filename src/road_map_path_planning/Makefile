include ../Makefile.conf

MODULE_NAME = Road Map Path Planning
MODULE_COMMENT = Plan routes from OpenStreetMap

LINK = g++
CFLAGS += -Wall

CXXFLAGS += -std=c++11
CXXFLAGS += -Wno-ignored-qualifiers

IFLAGS += \
	`pkg-config --cflags opencv` \
	-I/usr/include/python3.5

LFLAGS += \
	`python3.5-config --libs` `pkg-config --libs opencv` \
	-lglobal -lipc -lparam_interface -lvoice_interface_interface \
	-llocalize_ackerman_interface \
	-lmap_server_interface \
	-lgrid_mapping \
	-lmap_io \
	-lmap_interface \
	-lgps_xyz_interface \
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