include ../Makefile.conf

# Module name and description
MODULE_NAME = Neural Motion Planner
MODULE_COMMENT = neural motion planner

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

IFLAGS += -I/usr/include/python3.5 -I/usr/include/python3.6 -I/usr/include/python3.8

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
		-lcollision_detection -lcarmodel -lgsl -lgslcblas -lm -lford_escape_hybrid_interface -lmoving_objects_interface \
		-ltask_manager_interface -lvoice_interface_interface
			
UBUNTU_VERSION ="$(shell cat /etc/issue | grep Ubuntu | cut -c8-12)"
#$(warning now we reached ifeq UBUNTU_VERSION=$(UBUNTU_VERSION))
ifeq ($(UBUNTU_VERSION),"16.04")
	LFLAGS += `python3.5-config --libs`
else ifeq ($(UBUNTU_VERSION),"18.04")
	LFLAGS += `python3.6-config --libs`
else
	LFLAGS += `python3.8-config --embed --libs` -pthread
endif

SOURCES = neural_motion_planner_main.cpp neural_motion_planner.cpp \
		neural_motion_planner_optimizer.cpp global_state.cpp publisher_util.cpp util.cpp pose.cpp \
		neural_motion_planner_interface.c

PUBLIC_BINARIES = neural_motion_planner
PUBLIC_LIBRARIES = libneural_motion_planner_interface.a libneural_motion_planner_optimizer.a
PUBLIC_INCLUDES = neural_motion_planner_interface.h neural_motion_planner_message.h rrt_node.h robot_state.h command.h pose.h \
		neural_motion_planner_optimizer.h

TARGETS = libcarmodel neural_motion_planner libneural_motion_planner_interface.a libneural_motion_planner_optimizer.a

libcarmodel:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libcarmodel
clean: clean_libcarmodel
clean_libcarmodel:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libcarmodel

neural_motion_planner: neural_motion_planner_main.o neural_motion_planner.o \
	neural_motion_planner_optimizer.o global_state.o publisher_util.o util.o pose.o command.o neural_motion_planner_interface.o


libneural_motion_planner_interface.a: neural_motion_planner_interface.o

libneural_motion_planner_optimizer.a: neural_motion_planner_optimizer.o command.o

global_state: global_state.o

publisher_util: publisher_util.o

util: util.o

pose: pose.o

command: command.o

include ../Makefile.rules
