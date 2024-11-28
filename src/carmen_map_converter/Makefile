include ../Makefile.conf

MODULE_NAME = CARMEN MAP CONVERTER
MODULE_COMMENT = "Carmen Map Converter"

# Including Carmen and IPC Header
LFLAGS += -lglobal -lipc -lparam_interface

# Including Carmen and IPC Heade "prob_*" headers (* means all modules)
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -fopenmp
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -fopenmp 

# Including GTK
FLAGS += `pkg-config --cflags gtk+-2.0 gmodule-export-2.0`
IFLAGS += `pkg-config --cflags gtkglext-1.0`
LFLAGS += `pkg-config --libs gtk+-2.0 gmodule-export-2.0`
LFLAGS += `pkg-config --libs gtkglext-1.0`

# Including Mapper module	

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I/usr/include/bullet/ -I/usr/local/carmen_boost/include
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L/usr/local/carmen_boost/lib

LFLAGS+= -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system

#Lib_Neural_Mapper_py
IFLAGS += -I/usr/include/python3.5 -I/usr/include/python3.6 -I/usr/include/python3.8
UBUNTU_VERSION ="$(shell cat /etc/issue | grep Ubuntu | cut -c8-12)"
#$(warning now we reached ifeq UBUNTU_VERSION=$(UBUNTU_VERSION))
ifeq ($(UBUNTU_VERSION),"16.04")
	LFLAGS += `python3.5-config --libs`
else ifeq ($(UBUNTU_VERSION),"18.04")
	LFLAGS += `python3.6-config --libs`
else
	LFLAGS += `python3.8-config --embed --libs` -pthread
endif

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libsickldmrs2/include
# Required default libraries to comunicate with Carmen Core.
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libsickldmrs2 

LFLAGS += -lgrid_mapping -lmapper -lmap_io -lmap_util -lmap_interface -lipc -lprob_models -lm -lz -lglobal 		\
	-lparam_interface -llaser_interface -lipc -lmapper_interface -lobstacle_avoider_interface		\
	-llocalize_ackerman_interface -lmap_interface -lmapper -lipc		\
	-lultrasonic_filter_interface -lrotation_geometry -lvelodyne_interface	\
	-lfused_odometry_interface  -lstereo_velodyne -lstereo_velodyne_interface -lstereo_interface		\
	`pkg-config --libs opencv` -lgeometry -lmap_server_interface -lsimulator_ackerman_interface -lrddf_interface \
	-lparking_assistant_interface -llocalize_ackerman_core -lcarmodel -lgsl -lgslcblas -llocalize_ackerman_motion -ltask_manager_interface \
	-llaser_ldmrs_interface -llaser_ldmrs_utils -ldbscan \
	-lmoving_objects_interface

# Application specific include directories.
PCL_INC = $(wildcard /usr/local/include/pcl-*)
VTK_INC = $(wildcard /usr/local/include/vtk-5* /usr/include/vtk-5*)
IFLAGS += -I/usr/include/eigen3 -I $(PCL_INC) -I $(VTK_INC)

LFLAGS += -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search -lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization  -lboost_system -lboost_thread-mt -lrt -lboost_signals -lboost_system
LFLAGS +=  -lrddf_util -lcollision_detection -L/usr/lib64/libkml -lkmlbase -lkmldom -lkmlengine -lcarmen_gps_wrapper -lcarmen_util

CXXFLAGS += -std=c++14 

SOURCES = carmen_test_main.cpp ../mapper/mapper.cpp ../mapper/mapper_main.cpp ../mapper/message_interpolation.cpp \
		  ../mapper/convert_maps_from_float_to_double.cpp ../mapper/mapper_interface.c ../mapper/compute_velodyne_calibration_table.cpp\
		  ../mapper/neural_mapper_io.cpp ../mapper/neural_map.cpp ../mapper/lib_neural_mapper_py.cpp ../mapper/test_diff_map.cpp

TARGETS = carmen_test ../mapper/mapper ../mapper/libprob_models.a ../mapper/libmapper_interface.a ../mapper/libmapper.a\
 ../mapper/compute_velodyne_calibration_table ../mapper/test_diff_map

PUBLIC_BINARIES = carmen_test

carmen_test: carmen_test_main.o ../mapper/mapper.o ../mapper/mapper_main.o ../mapper/libmapper.a ../mapper/libmapper_interface.a $(CARMEN_HOME)/sharedlib/prob_models/libprob_models.a

include ../Makefile.rules
