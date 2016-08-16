include ../Makefile.conf

MODULE_NAME = Behavior Selector
MODULE_COMMENT = Change path planner algorithms and speed limits

LINK=g++
CFLAGS += -std=c99
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models

LFLAGS += $(INTERFACE_LIBS) -lglobal -lipc -lmap_interface -lparam_interface  \
	-lmapper_interface -lrddf_interface -lnavigator_ackerman_interface \
	-llocalize_ackerman_interface -lmap_server_interface -lcollision_detection -lmotion_planner_interface \
	-lobstacle_avoider_interface -lgrid_mapping -lmap_io -lmap_util \
	-lprob_models -lrotation_geometry

IFLAGS += -I/usr/include/eigen3 
IFLAGS += -I/usr/include/suitesparse -I/usr/include/eigen3 -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/

SOURCES = behavior_selector_interface.c behavior_selector_main.cpp behavior_selector.cpp

PUBLIC_INCLUDES =  behavior_selector_interface.h behavior_selector_messages.h
PUBLIC_LIBRARIES =  libbehavior_selector_interface.a

PUBLIC_BINARIES = behavior_selector

TARGETS =  behavior_selector libbehavior_selector_interface.a

behavior_selector: behavior_selector_main.o behavior_selector.o libbehavior_selector_interface.a

libbehavior_selector_interface.a: behavior_selector_interface.o

include ../Makefile.rules
