include ../Makefile.conf

# Module name and description
MODULE_NAME = CAR_PANEL_GUI
MODULE_COMMENT = Graphics User Interface for car panel 

LINK = g++

CXXFLAGS += -std=c++11
DEL_FILE = rm -f

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -I$(CARMEN_HOME)/src/car_panel_gui
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models 

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lglobal -lGLU -lipc -lz -lparam_interface -lmap_interface -llocalize_ackerman_interface -lsimulator_ackerman_interface \
	  -lobstacle_avoider_interface -lprob_models -llaser_interface -lmapper_interface -lnavigator_ackerman_interface \
	  -lglobal_graphics_qt -lrrt_planner_interface -lGL -lglut -lGLEW -lQtCore -lpthread -lfused_odometry_interface -lobstacle_avoider_interface \
	  -lmotion_planner_interface -lrobot_ackerman_interface -lbase_ackerman_interface

# Source code files (.c, .cpp) 
SOURCES	= 	character.cpp \
		speedometer.cpp \
		steering.cpp \
		lights.cpp \
		arrow.cpp \
		accelerator.cpp \
		without_time.cpp \
		panel.cpp \
		car_panel.cpp \
		panel_gui.cpp		

OBJECTS =	character.o \
		speedometer.o \
		steering.o \
		lights.o \
		arrow.o \
		accelerator.o \
		without_time.o \
		panel.o \
		car_panel.o 

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libcar_panel.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = car_panel_gui

TARGETS = car_panel_gui libcar_panel.a

PUBLIC_LIBRARIES_SO = libcar_panel.so

# rules


libcar_panel.a: $(OBJECTS)

car_panel_gui: $(OBJECTS) panel_gui.o


include ../Makefile.rules