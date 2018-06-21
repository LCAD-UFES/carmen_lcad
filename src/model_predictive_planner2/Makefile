include ../Makefile.conf

# Module name and description
MODULE_NAME = Model-Predictive Motion Planner
MODULE_COMMENT = Model-predictive motion planner

LINK = g++
CFLAGS += -fopenmp
CXXFLAGS += -fopenmp -std=c++0x
#CXXFLAGS += -DDEBUG_OBJECTS_PLOT
#CFLAGS += -msse3 -O4 -fopenmp
#CXXFLAGS += -std=c++0x -msse3 -O4 -fopenmp
#CFLAGS += -ffast-math -O3 -msse3 #-march=native
#CXXFLAGS += -std=c++0x -ffast-math -O3 -msse3 #-march=native

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcarmodel
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol

LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcarmodel

#G2O
IFLAGS += -I/usr/include/eigen3 
IFLAGS += -I/usr/include/suitesparse -I/usr/include/eigen3 -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/

#SUBDIRS += model
LFLAGS += 	-lgrid_mapping -lmap_io -lmap_util -lmap_interface -lparam_interface -lipc -lglobal -lmap_interface  -lprob_models -llocalize_ackerman_interface -lsimulator_ackerman_interface \
		-lrobot_ackerman_interface -lbase_ackerman_interface \
		-lnavigator_ackerman_interface -lobstacle_avoider_interface -lmapper_interface -lobstacle_distance_mapper_interface -lrrt_planner_interface \
		-lmap_server_interface -lbehavior_selector_interface  -lrotation_geometry \
		-L/usr/lib64/libkml -lkmlbase -lkmldom -lkmlengine -lcarmen_gps_wrapper \
		-lrddf_interface -lmotion_planner_interface -lnavigator_gui_interface -llocalize_ackerman_core \
		-lcollision_detection -lcarmodel -lgsl -lgslcblas -lm -lford_escape_hybrid_interface -lmoving_objects_interface
			
			
SOURCES = model_predictive_planner_main.cpp model_predictive_planner.cpp trajectory_lookup_table.cpp \
		model_predictive_planner_optimizer.cpp global_state.cpp publisher_util.cpp util.cpp pose.cpp command.cpp

PUBLIC_BINARIES = model_predictive_planner
PUBLIC_LIBRARIES = 

TARGETS = libcarmodel model_predictive_planner

libcarmodel:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libcarmodel
clean: clean_libcarmodel
clean_libcarmodel:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libcarmodel

model_predictive_planner: model_predictive_planner_main.o model_predictive_planner.o trajectory_lookup_table.o \
	model_predictive_planner_optimizer.o global_state.o publisher_util.o util.o pose.o command.o

trajectory_lookup_table: trajectory_lookup_table.o

global_state: global_state.o

publisher_util: publisher_util.o

util: util.o

pose: pose.o

command: command.o

include ../Makefile.rules
