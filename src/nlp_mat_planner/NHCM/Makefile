
include ../../Makefile.conf

# NHCM creator and visualizer
MODULE_NAME = NHCM for Offroad path planner A-star 
MODULE_COMMENT = NHCM heuristic matrix for Offroad path planner A-star 

LINK ?= g++
CFLAGS += -Wno-deprecated-declarations
CXXFLAGS += -std=c++11 -Wno-deprecated-declarations
#LINK = g++ -g
#CFLAGS += -Wall -g
#CXXFLAGS = -std=c++11 -g
CFLAGS += -fopenmp
CXXFLAGS += -fopenmp

IFLAGS += -I/usr/include/eigen3 
IFLAGS += -I$(ASTRO_HOME)/sharedlib/prob_models -I$(ASTRO_HOME)/sharedlib/libcarmodel -I$(ASTRO_HOME)/sharedlib/libcontrol -I/home/lcad/astro/include
LFLAGS += -L$(ASTRO_HOME)/sharedlib/prob_models -L$(ASTRO_HOME)/sharedlib/libcarmodel -L$(ASTRO_HOME)/sharedlib/libcontrol

LFLAGS += -lglobal -lipc -lparam_interface -llocalize_ackerman_interface -lrddf_interface -lobstacle_distance_mapper_interface \
    -lrobot_ackerman_interface -lsimulator_ackerman_interface -lbase_ackerman_interface \
	-lmap_server_interface -lgrid_mapping -lmap_io -lmap_interface -lcollision_detection -lnavigator_ackerman_interface \
	-lcarmodel `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv` -lmapper_interface -lgsl -lgslcblas -loffroad_planner_interface -lroute_planner_interface \
	-ltask_manager_interface -lastro_util

CASADI_DIR=$(shell /usr/bin/env python3 -c "import casadi; print(casadi.__path__[0])")
LFLAGS += -L$(CASADI_DIR) -lcasadi -lstdc++

CXXFLAGS += -DUSE_CASADI
LFLAGS += -ltrailer_nlp

#Astro Global Graphic Library and Gtk+2 Graphics Libraries

IFLAGS += \
	`pkg-config --cflags gtk+-2.0 gmodule-export-2.0` \
	`pkg-config --cflags gtkglext-1.0`
LFLAGS += \
	-lglobal_graphics \
	`pkg-config --libs gtk+-2.0 gmodule-export-2.0` \
	`pkg-config --libs gtkglext-1.0`

SOURCES = ../reeds_sheep.cpp cost_matrix.cpp cost_matrix_trailer.cpp cost_matrix_trailer2.cpp ../trailer_analytical_expansion.cpp #planner_study.cpp
TARGETS = cost_matrix cost_matrix_trailer cost_matrix_trailer2 #planner_study

cost_matrix: cost_matrix.o ../reeds_sheep.o
cost_matrix_trailer: cost_matrix_trailer.o ../trailer_analytical_expansion.o
#cost_matrix_trailer: cost_matrix_trailer.o ../trailer_analytical_expansion.o
cost_matrix_trailer2: cost_matrix_trailer2.o ../trailer_analytical_expansion.o

#planner_study: planner_study.o ../trailer_analytical_expansion.o

include ../../Makefile.rules

