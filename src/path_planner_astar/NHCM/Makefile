
include ../../Makefile.conf

# NHCM creator and visualizer
MODULE_NAME = NHCM for Path Planner A-star
MODULE_COMMENT = NHCM heuristic matrix for Path Planner Astar

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

SOURCES = ../reeds_sheep.cpp cost_matrix.cpp cost_matrix_visualizer.cpp 
PUBLIC_BINARIES = cost_matrix cost_matrix_visualizer 
TARGETS = cost_matrix cost_matrix_visualizer 

cost_matrix: cost_matrix.o ../reeds_sheep.o
cost_matrix_visualizer: cost_matrix_visualizer.o ../reeds_sheep.o

include ../../Makefile.rules

