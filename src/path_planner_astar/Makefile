
include ../Makefile.conf

# Path planner A-star
MODULE_NAME = Path Planner A-star
MODULE_COMMENT = Path Planning using A-star algorithm

LINK = g++
CFLAGS += -Wall -g 
CXXFLAGS = -std=c++11 -ggdb

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -I$(CARMEN_HOME)/sharedlib/libcarmodel -I$(CARMEN_HOME)/sharedlib/libcontrol -I/home/lcad/carmen_lcad/include
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -L$(CARMEN_HOME)/sharedlib/libcarmodel -L$(CARMEN_HOME)/sharedlib/libcontrol

LFLAGS += -lglobal -lipc -lparam_interface -llocalize_ackerman_interface -lrddf_interface -lobstacle_distance_mapper_interface \
	-lobstacle_avoider_interface -lrobot_ackerman_interface -lsimulator_ackerman_interface -lbase_ackerman_interface \
	-lmap_server_interface -lgrid_mapping -lmap_io -lmap_interface -lcollision_detection -lnavigator_ackerman_interface -lcarmodel `pkg-config --libs opencv` -lmapper_interface -lgsl -lgslcblas -loffroad_planner_interface -lroute_planner_interface

#Carmen Global Graphic Library and Gtk+2 Graphics Libraries

IFLAGS += \
	`pkg-config --cflags gtk+-2.0 gmodule-export-2.0` \
	`pkg-config --cflags gtkglext-1.0`
LFLAGS += \
	-lglobal_graphics \
	`pkg-config --libs gtk+-2.0 gmodule-export-2.0` \
	`pkg-config --libs gtkglext-1.0`

SOURCES = path_planner_astar_main.cpp path_planner_astar.cpp reeds_sheep.cpp planning.cpp offroad_planner_distance_map.cpp 

PUBLIC_BINARIES = path_planner_astar 
TARGETS = path_planner_astar 

path_planner_astar: path_planner_astar_main.o path_planner_astar.o reeds_sheep.o planning.o offroad_planner_distance_map.o

include ../Makefile.rules

