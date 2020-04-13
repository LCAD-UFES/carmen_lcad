include ../Makefile.conf

# Module name and description
MODULE_NAME = RDDF 
MODULE_COMMENT = Rddf Builder And Player

LINK = g++

CFLAGS += -Wall -g 
CXXFLAGS += -std=c++11
LFLAGS += -lparam_interface -lipc -lglobal \
	-llocalize_ackerman_interface \
	-lobstacle_avoider_interface -lsimulator_ackerman_interface -ltraffic_light_interface \
	-lmoving_objects_interface \
	-lmap_server_interface -lgrid_mapping -lmap_io -lmap_interface -lrddf_interface -lrddf_util -lrddf_index -lcollision_detection \
	-lcarmen_gps_wrapper -lgsl -lgslcblas -lm -lvoice_interface_interface `python3.5-config --libs` 
LFLAGS += -L/usr/lib64/libkml -lkmlbase -lkmldom -lkmlengine

IFLAGS += -I/usr/include/eigen3 
IFLAGS += -I/usr/include/suitesparse -I/usr/include/eigen3 -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/
IFLAGS += -I/usr/include/python3.5

SOURCES = frenet_path_planner_main.cpp
TARGETS = frenet_path_planner libfrenet_path_planner_interface.a

PUBLIC_INCLUDES = frenet_path_planner_interface.h  frenet_path_planner_messages.h
PUBLIC_LIBRARIES = libfrenet_path_planner_interface.a
PUBLIC_BINARIES = frenet_path_planner

#PUBLIC_LIBRARIES_SO = librddf_interface.so
#ifndef NO_PYTHON
#TARGETS += librddf_interface.so.1
#endif

frenet_path_planner: frenet_path_planner_main.o libfrenet_path_planner_interface.a

libfrenet_path_planner_interface.a: frenet_path_planner_interface.o

# rules
include ../Makefile.rules
