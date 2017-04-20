include ../Makefile.conf

MODULE_NAME = Moving obstacle detector

MODULE_COMMENT = Moving obstacle detector based on subtraction of cost maps.

LINK = g++

CFLAGS += -Wall -std=c99
CXXFLAGS += -Wall -std=c++98

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models  -fopenmp
LFLAGS += -lglobal -lipc -llaser_interface -lGL -lglut -lGLU -lparam_interface \
	 -L$(CARMEN_HOME)/sharedlib/prob_models -lprob_models \
	 -lmap_interface -lrotation_geometry -lfused_odometry_interface -lbase_ackerman_interface -lmap_server_interface \
	 -lmapper_interface -lmoving_objects_interface -llocalize_ackerman_interface -lobstacle_distance_mapper_interface `pkg-config --cflags opencv` `pkg-config --libs opencv`

SOURCES = moving_obstacle_detector_main.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 

# If you set graphics support in configure.
ifndef NO_GRAPHICS
#Carmen Global Graphic Library and Gtk+2 Graphics Libraries
LFLAGS += -lglobal_graphics
IFLAGS += `pkg-config --cflags gtk+-2.0 gmodule-export-2.0`
IFLAGS += `pkg-config --cflags gtkglext-1.0`
LFLAGS += `pkg-config --libs gtk+-2.0 gmodule-export-2.0`
LFLAGS += `pkg-config --libs gtkglext-1.0`
endif

TARGETS = moving_obstacle_detector

moving_obstacle_detector: moving_obstacle_detector_main.o

include ../Makefile.rules
