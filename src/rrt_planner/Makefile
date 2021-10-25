include ../Makefile.conf

# Module name and description
MODULE_NAME = RRT Planner
MODULE_COMMENT = Rapidly-exploring Random Trees Planner

LINK = g++
CFLAGS += -fopenmp
CXXFLAGS += -std=c++0x

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcarmodel/
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol/

SUBDIRS += model message util path_follower
LFLAGS += 	-lgrid_mapping -lmap_io -lmap_util -lmap_interface -lparam_interface -lipc -lglobal -lmap_interface  -lprob_models -llocalize_ackerman_interface -lsimulator_ackerman_interface \
		-lrobot_ackerman_interface -lbase_ackerman_interface \
		-lnavigator_ackerman_interface -lobstacle_avoider_interface -lmapper_interface -lrrt_planner_interface \
		-lmap_server_interface -lbehavior_selector_interface  -lrotation_geometry \
		-L/usr/lib64/libkml -lkmlbase -lkmldom -lkmlengine -lcarmen_gps_wrapper \
		-lrddf_interface -lmotion_planner_interface -lnavigator_gui_interface -llocalize_ackerman_core -lcarmodel -ltask_manager_interface \
		-lcollision_detection -lgsl -lgslcblas -lm
			
			
SOURCES = rrt_main.cpp rrt.cpp path_follower_main.cpp rs.cpp data_collector_main.cpp rrt_parking.cpp rrt_lane.cpp

PUBLIC_BINARIES = rrt_planner rrt_path_follower data_collector
PUBLIC_LIBRARIES = librrt_util_wrapper.a librrt_planner_interface.a

TARGETS = rrt_planner rrt_path_follower librrt_util_wrapper.a librrt_planner_interface.a data_collector

librrt_model.a: model/tree.o model/rrt_node.o model/command.o model/robot_state.o model/pose.o model/global_state.o

librrt_util.a: util/publisher_util.o util/util.o util/obstacle_detection.o util/dijkstra.o util/ackerman.o util/lane.o rs.o util/cost_function.o

librrt_util_wrapper.a: util/lane.o util/carmen_rrt_util.o librrt_model.a

librrt_planner_interface.a: message/rrt_planner_interface.o

data_collector: data_collector_main.o

rrt_planner: rrt.o rrt_main.o rrt_parking.o rrt_lane.o rs.o librrt_model.a librrt_util.a
rrt_path_follower: path_follower_main.o path_follower/follower.o path_follower/path_follower_ackerman.o librrt_model.a librrt_util.a

include ../Makefile.rules
