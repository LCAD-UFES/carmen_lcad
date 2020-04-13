include ../Makefile.conf

MODULE_NAME = RDDF 
MODULE_COMMENT = Rddf Builder And Player

LINK = g++
CFLAGS += -Wall -g 
CXXFLAGS += -std=c++11

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -I$(CARMEN_HOME)/sharedlib/libcarmodel -I$(CARMEN_HOME)/sharedlib/libcontrol
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -L$(CARMEN_HOME)/sharedlib/libcarmodel -L$(CARMEN_HOME)/sharedlib/libcontrol

LFLAGS += -lparam_interface -lipc -lglobal -llocalize_ackerman_interface -lrddf_interface -lrddf_util -lprob_models -lobstacle_distance_mapper_interface \
	-lobstacle_avoider_interface -lrobot_ackerman_interface -lsimulator_ackerman_interface -lcollision_detection \
	-lbase_ackerman_interface -lmap_server_interface -lgrid_mapping -lmap_io -lmap_interface \
	-lnavigator_ackerman_interface -lcarmodel `pkg-config --libs opencv`

SOURCES = path_planner_main.cpp hybrid_A_star.cpp reeds_sheep.cpp

PUBLIC_BINARIES = path_planner

TARGETS = path_planner

path_planner: path_planner_main.o hybrid_A_star.o reeds_sheep.o

include ../Makefile.rules
