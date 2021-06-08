include ../Makefile.conf

MODULE_NAME = Behavior Selector
MODULE_COMMENT = Change path planner algorithms and speed limits

LINK=g++ -pg
CFLAGS += -std=c99 -pg
CXXFLAGS += -std=c++11 -pg

IFLAGS += \
	-I$(CARMEN_HOME)/sharedlib/prob_models \
	-I/usr/include/suitesparse \
	-I/usr/include/eigen3 \
	-I/usr/local/include/EXTERNAL/csparse/ \
	-I/usr/local/include/g2o/

IFLAGS += -I/usr/include/python3.6 -I/usr/include/python3.5

LFLAGS += \
	-L$(CARMEN_HOME)/sharedlib/prob_models -fopenmp \
	-lglobal -lipc \
	-lmap_interface \
	-lparam_interface \
	-lrddf_index \
	-lrddf_interface \
	-lrddf_util \
	-lnavigator_ackerman_interface \
	-llocalize_ackerman_interface \
	-lmap_server_interface \
	-lcollision_detection \
	-lmotion_planner_interface \
	-lobstacle_avoider_interface \
	-lobstacle_distance_mapper_datmo \
	-lmap_server_interface \
	-lmapper_interface \
	-lgrid_mapping \
	-lmap_io \
	-lmap_util \
	-lprob_models \
	-lmap_interface \
	-lrotation_geometry \
	-lsimulator_ackerman_interface \
	-lobstacle_distance_mapper_interface \
	-ludatmo \
	-lmoving_objects_interface \
	-lvoice_interface_interface \
	-lfrenet_path_planner_interface \
	-lroute_planner_interface \
	-lcarmodel \
	-lgsl -lgslcblas \
	`pkg-config --libs opencv`
	
UBUNTU_VERSION ="$(shell cat /etc/issue | grep Ubuntu | cut -c8-12)"
#$(warning now we reached ifeq UBUNTU_VERSION=$(UBUNTU_VERSION))
ifeq ($(UBUNTU_VERSION),"16.04")
	LFLAGS += `python3.5-config --libs`
else
	LFLAGS += `python3.6-config --libs`
endif

LFLAGS += -L/usr/lib64/libkml -lkmlbase -lkmldom -lkmlengine -lcarmen_gps_wrapper

ifeq ("$(CARMEN_LOGGING)","ON")
CFLAGS += \
	-DBOOST_LOG_DYN_LINK
LFLAGS += \
	-lboost_log_setup \
	-lboost_log \
	-lboost_thread \
	-lboost_system
endif

SOURCES = \
	behavior_selector_interface.c \
	behavior_selector_main.cpp \
	behavior_selector_set_goal_list.cpp \
	behavior_selector_set_path.cpp \
	behavior_selector_set_goal_velocity.cpp \
	behavior_selector_mot.cpp \
	behavior_selector_state_machine.cpp \
	SampleFilter.cpp

PUBLIC_INCLUDES = \
	behavior_selector_interface.h \
	behavior_selector_messages.h

PUBLIC_LIBRARIES = \
	libbehavior_selector_interface.a

PUBLIC_BINARIES = \
	behavior_selector

TARGETS = \
	behavior_selector \
	libbehavior_selector_interface.a

# If you set graphics support in configure.
ifndef NO_GRAPHICS
#Carmen Global Graphic Library and Gtk+2 Graphics Libraries
IFLAGS += \
	`pkg-config --cflags gtk+-2.0 gmodule-export-2.0` \
	`pkg-config --cflags gtkglext-1.0`
LFLAGS += \
	-lglobal_graphics \
	`pkg-config --libs gtk+-2.0 gmodule-export-2.0` \
	`pkg-config --libs gtkglext-1.0`
endif


behavior_selector: behavior_selector_main.o behavior_selector_set_goal_list.o \
	behavior_selector_set_goal_velocity.o behavior_selector_set_path.o behavior_selector_mot.o \
	behavior_selector_state_machine.o SampleFilter.o libbehavior_selector_interface.a 

libbehavior_selector_interface.a: behavior_selector_interface.o


include ../Makefile.rules
