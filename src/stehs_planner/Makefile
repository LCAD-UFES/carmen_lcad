include ../Makefile.conf

MODULE_NAME = STEHS MOTION PLANNER
MODULE_COMMENT = Space Time Exploration Guided Heuristic Search Motion Planner 

LINK = g++
CXXFLAGS += -std=c++0x

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcarmodel
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcontrol
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcarmodel

LFLAGS += 	-lgrid_mapping -lmap_io -lmap_util -lmap_interface -lparam_interface -lipc -lglobal -lmap_interface  -lprob_models -lobstacle_distance_mapper_interface -llocalize_ackerman_interface -lsimulator_ackerman_interface \
		-lrobot_ackerman_interface -lbase_ackerman_interface \
		-lnavigator_ackerman_interface -lobstacle_avoider_interface -lmapper_interface -lrrt_planner_interface \
		-lmap_server_interface -lbehavior_selector_interface \
		-lrddf_interface -lmotion_planner_interface -lnavigator_gui_interface -llocalize_ackerman_core \
		-lcollision_detection -lm -lford_escape_hybrid_interface `pkg-config --cflags opencv` `pkg-config --libs opencv` -lcarmodel


SOURCES = stehs_planner_main.cpp stehs_planner.cpp CircleNode.cpp Circle.cpp StateNode.cpp

PUBLIC_BINARIES = stehs_planner
PUBLIC_LIBRARIES =

TARGETS = libcarmodel stehs_planner


libcarmodel:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libcarmodel
clean: clean_libcarmodel
clean_libcarmodel:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libcarmodel



stehs_planner: stehs_planner_main.o stehs_planner.o CircleNode.o Circle.o StateNode.o

include ../Makefile.rules
