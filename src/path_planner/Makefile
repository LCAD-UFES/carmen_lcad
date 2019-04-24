include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = PATH PLANNER
MODULE_COMMENT = Path Planer

LINK = g++

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models

LFLAGS += $(INTERFACE_LIBS) -lglobal -lipc \
	-lmap_interface -lgrid_mapping -lmapper_interface \
	-lparam_interface -lX11 \
	-llocalize_ackerman_interface  -lmap_io -lprob_models -lobstacle_distance_mapper_interface\
	-lsimulator_ackerman_interface -lobstacle_avoider_interface -lrddf_interface -lrobot_ackerman_interface \
	-lmotion_planner_interface -lcollision_detection -lmap_server_interface -lbehavior_selector_interface -lrotation_geometry  -lrddf_interface
CXXFLAGS += -std=c++0x 

SOURCES = navigator_astar.cpp conventional_ackerman.c conventional_astar_ackerman.cpp rs.cpp FH/fh.cpp FH/fib.cpp MessageControl.cpp navigator_ackerman_ipc.c \
	navigator_astar_interface.c trajectory_ackerman.c navigator_precomputed_cost_ackerman.c path_planner_interface.c

PUBLIC_INCLUDES = path_planner_messages.h path_planner_interface.h navigator_astar_interface.h navigator_astar.hpp MessageControl.hpp
PUBLIC_LIBRARIES = libnavigator_astar_interface.a libconventional_ackerman.a libpath_planner_interface.a
PUBLIC_BINARIES = path_planner

## 
## IMPORTANT: The 'path_planner' program is not being compiled because the struct 'carmen_navigator_config_t' defined at
## 'navigator_ackerman/navigator_ackerman.h' has changed and the source of the program was not updated.
## 
TARGETS =  libprob_models.a libnavigator_astar_interface.a libpath_planner_interface.a libconventional_ackerman.a navigator_precomputed_cost_ackerman startgoal_simulator # path_planner 

PUBLIC_LIBRARIES_SO = libnavigator_astar_interface.so
ifndef NO_PYTHON
TARGETS += libnavigator_astar_interface.so.1
endif

ifndef NO_TCPD
LFLAGS += -lwrap -lnsl
endif

libpath_planner_interface.a: path_planner_interface.o

path_planner: navigator_astar.o navigator_ackerman_ipc.o conventional_ackerman.o conventional_astar_ackerman.o rs.o FH/fh.o FH/fib.o MessageControl.o \
	trajectory_ackerman.o libnavigator_astar_interface.a

navigator_precomputed_cost_ackerman:  navigator_precomputed_cost_ackerman.o FH/fh.o

libnavigator_astar_interface.a: navigator_astar_interface.o

libnavigator_astar_interface.so.1: navigator_astar_interface.o

libconventional_ackerman.a: 	conventional_ackerman.o MessageControl.o trajectory_ackerman.o

followpath_ackerman: followpath_ackerman.o libnavigator_astar_interface.a

startgoal_simulator: startgoal_simulator.o libnavigator_astar_interface.a

# rules

libprob_models.a:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/prob_models/

clean: clean_prob_models

clean_prob_models:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/prob_models/

include $(CARMEN_HOME)/src/Makefile.rules
