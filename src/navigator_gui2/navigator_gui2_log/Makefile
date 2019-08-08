include ../../Makefile.conf

# Module name and description
MODULE_NAME = NAVIGATOR GUI 2 LOG
MODULE_COMMENT = Carmen Navigation Tool Log

LINK = g++

CFLAGS += -fopenmp

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src

# Required default libraries to comunicate with Carmen Core
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -I../car_panel_gui/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -L../car_panel_gui/

IFLAGS += `pkg-config --cflags opencv`

LFLAGS += -L/usr/local/lib -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -lparam_interface -lipc -lglobal -lmap_graphics -lglobal_graphics -lmap_interface -lprob_models \
		  -lgrid_mapping -lbase_ackerman_interface -lgeometry -lobstacle_avoider_interface \
		  -ldynamic_object_detector_interface -lmapper_interface -lcollision_detection \
		  -lsimulator_ackerman_interface -llocalize_ackerman_interface -lnavigator_ackerman_interface \
		  -lfused_odometry_interface -lmap_io -lrddf_interface -lmap_server_interface -lbehavior_selector_interface \
		  -lrotation_geometry -lcar_panel -lrobot_ackerman_interface -lparking_assistant_interface \
		  -lglm -lGL -lGLU -lglut -ltf -lmoving_objects_interface -llane_detector_interface -lmodel_predictive_planner_interface `pkg-config --libs opencv`
LFLAGS += -L/usr/lib64/libkml -lrddf_util -lkmlbase -lkmldom -lkmlengine -lcarmen_gps_wrapper

CFLAGS += -Wno-unknown-pragmas
CXXFLAGS += -Wno-unknown-pragmas

# Source code files (.c, .cpp) 
SOURCES = image_ready.cpp

#Carmen Global Graphic Library and Gtk+2 Graphics Libraries
LFLAGS += -lglobal_graphics
IFLAGS += `pkg-config --cflags gtk+-2.0 gmodule-export-2.0`
IFLAGS += `pkg-config --cflags gtkglext-1.0`
LFLAGS += `pkg-config --libs gtk+-2.0 gmodule-export-2.0`
LFLAGS += `pkg-config --libs gtkglext-1.0`

image_ready: image_ready.o 

# rules

include ../../Makefile.rules
