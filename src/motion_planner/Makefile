include ../Makefile.conf

MODULE_NAME = Motion Planner
MODULE_COMMENT = The real motion planner!

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models

IFLAGS +=
LFLAGS += $(INTERFACE_LIBS) -lglobal -lipc \
	-lgeometry -lmap_interface \
	-lparam_interface -lX11 \
	-llocalize_ackerman_interface  -lgrid_mapping_interface -lprob_models \
	-lsimulator_ackerman_interface -lobstacle_avoider_interface -lrddf_interface -lnavigator_ackerman_interface \
	-lbehavior_selector_interface -lrobot_ackerman_interface

SOURCES = motion_planner_interface.c motion_planner_main.c motion_planner.c

PUBLIC_INCLUDES = motion_planner_interface.h motion_planner_messages.h motion_planner.h
PUBLIC_LIBRARIES =  libmotion_planner_interface.a

PUBLIC_BINARIES = motion_planner

TARGETS =  libprob_models.a motion_planner libmotion_planner_interface.a

motion_planner: motion_planner_main.o motion_planner.o libmotion_planner_interface.a

libmotion_planner_interface.a: motion_planner_interface.o

# rules

libprob_models.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/prob_models/

clean: clean_prob_models

clean_prob_models:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/prob_models/
	
include ../Makefile.rules
