include ../Makefile.conf

# Module name and description
MODULE_NAME = RDDF 
MODULE_COMMENT = Rddf Builder And Player

LINK = g++

CFLAGS += -Wall -g 
CXXFLAGS += -std=c++11
LFLAGS += -lparam_interface -lipc -lglobal -lrddf_interface \
	-lcarmen_gps_wrapper  -llocalize_ackerman_interface \
	-lkinect_interface \
	-lobstacle_avoider_interface -lrobot_ackerman_interface -lsimulator_ackerman_interface -ltraffic_light_interface \
	-lcollision_detection -lmoving_objects_interface \
	-lmap_server_interface -lgrid_mapping -lmap_io -lmap_interface \
	-lgsl -lgslcblas -lm -lvoice_interface_interface 
LFLAGS += -L/usr/lib64/libkml -lkmlbase -lkmldom -lkmlengine

UBUNTU_VERSION ="$(shell cat /etc/issue | grep Ubuntu | cut -c8-12)"
#$(warning now we reached ifeq UBUNTU_VERSION=$(UBUNTU_VERSION))
ifeq ($(UBUNTU_VERSION),"16.04")
	LFLAGS += `python3.5-config --libs`
else ifeq ($(UBUNTU_VERSION),"18.04")
	LFLAGS += `python3.6-config --libs`
else
	LFLAGS += `python3.8-config --embed --libs` -pthread
endif


IFLAGS += -I/usr/include/eigen3 
IFLAGS += -I/usr/include/suitesparse -I/usr/include/eigen3 -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/
IFLAGS += -I/usr/include/python3.5 -I/usr/include/python3.6 -I/usr/include/python3.8

SOURCES = rddf_interface.c rddf_play_main.cpp rddf_index.cpp rddf_build_main.cpp rddf_util.cpp rddf_play_annotations.cpp rddf_by_road_map.cpp rddf_resample.cpp
SOURCES += test_update_annotation.cpp

PUBLIC_INCLUDES = rddf_interface.h rddf_messages.h rddf_util.h rddf_index.h
PUBLIC_LIBRARIES = librddf_interface.a librddf_util.a librddf_index.a
PUBLIC_BINARIES = rddf_build rddf_play rddf_annotation_manager rddf_convert_from_kml_to_txt rddf_resample
TARGETS = librddf_interface.a rddf_build rddf_play rddf_annotation_manager librddf_util.a librddf_index.a rddf_convert_from_kml_to_txt rddf_resample
TARGETS += test_update_annotation

PUBLIC_LIBRARIES_SO = librddf_interface.so
ifndef NO_PYTHON
TARGETS += librddf_interface.so.1
endif

librddf_index.a: rddf_index.o
librddf_util.a: rddf_play_annotations.o rddf_util.o rddf_index.o rddf_by_road_map.o
librddf_interface.a: rddf_interface.o
librddf_interface.so.1: rddf_interface.o
rddf_play: rddf_index.o rddf_play_main.o rddf_interface.o rddf_play_annotations.o rddf_by_road_map.o rddf_util.o
rddf_build: rddf_build_main.o rddf_util.o rddf_index.o
rddf_annotation_manager: rddf_annotation_manager.o rddf_interface.o 
rddf_convert_from_kml_to_txt: rddf_convert_from_kml_to_txt.o rddf_util.o rddf_index.o
rddf_resample: rddf_resample.o
test_update_annotation: test_update_annotation.o rddf_interface.o rddf_util.o rddf_index.o

# rules
include ../Makefile.rules
