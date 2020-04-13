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
	-lrddf_util \
	-lgps_xyz_interface \
	-lGL \
	-lglut \
	-lGLU \
	-lnavigator_ackerman_interface -lrddf_interface

SOURCES = \
	road_map_path_planning_main.cpp \
	road_map_path_planning_utils.cpp \
	call_iara_app_main.cpp 


PUBLIC_INCLUDES = \
	road_map_path_planning_utils.h \
	call_iara_app_messages.h
	

#PUBLIC_LIBRARIES = libroad_map_path_planning_utils.a 
	

PUBLIC_BINARIES = \
	road_map_path_planning_main\
	call_iara_main
	

TARGETS = road_map_path_planning_main \
		call_iara_main #libroad_map_path_planning_utils.a
		  

road_map_path_planning_main: road_map_path_planning_main.o road_map_path_planning_utils.o
call_iara_main: call_iara_app_main.o road_map_path_planning_utils.o

#road_map_path_planning_utils.a: road_map_path_planning_utils.o

include ../Makefile.rules