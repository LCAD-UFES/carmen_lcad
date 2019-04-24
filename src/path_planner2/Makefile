include ../Makefile.conf

MODULE_NAME = RDDF 
MODULE_COMMENT = Rddf Builder And Player

LINK = g++
CFLAGS += -Wall -g 
CXXFLAGS += -std=c++11

IFLAGS += -I/usr/include/eigen3
IFLAGS += -I/usr/include/suitesparse -I/usr/include/eigen3 -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/
IFLAGS += -I/usr/include/python3.5
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models
LFLAGS += -L/usr/lib64/libkml -lkmlbase -lkmldom -lkmlengine

LFLAGS += -lparam_interface -lipc -lglobal -lrddf_interface -lcarmen_gps_wrapper  -llocalize_ackerman_interface	-lkinect_interface -lprob_models -lobstacle_distance_mapper_interface \
	-lobstacle_avoider_interface -lrobot_ackerman_interface -lsimulator_ackerman_interface -ltraffic_light_interface -lcollision_detection \
	-lmoving_objects_interface -lbase_ackerman_interface -lmap_server_interface -lgrid_mapping -lmap_io -lmap_interface \
	-lgsl -lgslcblas -lm -lvoice_interface_interface `python3.5-config --libs` `pkg-config --libs opencv`

SOURCES = rddf_interface.c rddf_play_main.cpp rddf_index.cpp rddf_build_main.cpp rddf_util.cpp stehs_planner.cpp

PUBLIC_INCLUDES = rddf_interface.h rddf_messages.h rddf_util.h rddf_index.h

PUBLIC_LIBRARIES = librddf_interface.a librddf_util.a librddf_index.a

PUBLIC_BINARIES = rddf_build path_planner2 rddf_annotation_manager rddf_convert_from_kml_to_txt

TARGETS = librddf_interface.a rddf_build path_planner2 rddf_annotation_manager librddf_util.a librddf_index.a rddf_convert_from_kml_to_txt

PUBLIC_LIBRARIES_SO = librddf_interface.so
ifndef NO_PYTHON
TARGETS += librddf_interface.so.1
endif

librddf_index.a: rddf_index.o

librddf_util.a: rddf_util.o

librddf_interface.a: rddf_interface.o

librddf_interface.so.1: rddf_interface.o

path_planner2: rddf_index.o rddf_play_main.o rddf_interface.o rddf_util.o stehs_planner.o

rddf_build: rddf_build_main.o rddf_util.o

rddf_annotation_manager: rddf_annotation_manager.o rddf_interface.o 

rddf_convert_from_kml_to_txt: rddf_convert_from_kml_to_txt.o rddf_util.o

include ../Makefile.rules
