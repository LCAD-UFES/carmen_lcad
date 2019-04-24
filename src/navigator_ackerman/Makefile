include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = NAVIGATOR ACKERMAN
MODULE_COMMENT = The motion planner

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models

IFLAGS +=
LFLAGS += $(INTERFACE_LIBS) -lglobal -lipc \
	-lgeometry -lmap_interface \
	-lparam_interface -lX11 \
	-llocalize_ackerman_interface  -lmapper_interface -lgrid_mapping -lprob_models \
	-lsimulator_ackerman_interface -lobstacle_avoider_interface -lrddf_interface -lrobot_ackerman_interface \
	-lmotion_planner_interface -lcollision_detection -lmap_server_interface -lbehavior_selector_interface
CFLAGS += #-pg

ifeq (Fedora release 14 (Laughlin),$(SO_DISTRO))
  LFLAGS += -lX11
endif

SOURCES = navigator_ackerman.c conventional_ackerman.c conventional_astar_ackerman.c rs.cpp path_optimization_ackerman.c FH/fh.c FH/fib.c planner_ackerman.c navigator_ackerman_ipc.c \
	navigator_ackerman_interface.c trajectory_ackerman.c map_ackerman_modify.c \
	navigator_ackerman_test.c \
navigator_precomputed_cost_ackerman.c

PUBLIC_INCLUDES = navigator_ackerman_messages.h navigator_ackerman_interface.h navigator_ackerman.h planner_ackerman_interface.h
PUBLIC_LIBRARIES =  libnavigator_ackerman_interface.a libconventional_ackerman.a

PUBLIC_BINARIES = navigator_ackerman

TARGETS =  libprob_models.a libnavigator_ackerman_interface.a navigator_ackerman navigator_ackerman_test navigator_precomputed_cost_ackerman \
 libconventional_ackerman.a

PUBLIC_LIBRARIES_SO = libnavigator_ackerman_interface.so
ifndef NO_PYTHON
TARGETS += libnavigator_ackerman_interface.so.1
endif

ifndef NO_TCPD
LFLAGS += -lwrap -lnsl
endif

navigator_ackerman: navigator_ackerman.o navigator_ackerman_ipc.o conventional_ackerman.o conventional_astar_ackerman.o rs.o path_optimization_ackerman.o FH/fh.o FH/fib.o planner_ackerman.o \
	trajectory_ackerman.o map_ackerman_modify.o libnavigator_ackerman_interface.a

navigator_precomputed_cost_ackerman:  navigator_precomputed_cost_ackerman.o FH/fh.o

libnavigator_ackerman_interface.a: navigator_ackerman_interface.o

libnavigator_ackerman_interface.so.1: navigator_ackerman_interface.o

libconventional_ackerman.a: 	conventional_ackerman.o planner_ackerman.o trajectory_ackerman.o\
 map_ackerman_modify.o

navigator_ackerman_test: navigator_ackerman_test.o navigator_ackerman_interface.o

random_nav_ackerman: random_nav_ackerman.o libnavigator_ackerman_interface.a

followpath_ackerman: followpath_ackerman.o libnavigator_ackerman_interface.a

# rules

libprob_models.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/prob_models/

clean: clean_prob_models

clean_prob_models:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/prob_models/
	
include $(CARMEN_HOME)/src/Makefile.rules
