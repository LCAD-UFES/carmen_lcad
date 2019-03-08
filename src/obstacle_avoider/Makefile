include ../Makefile.conf

# voice commands
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libvoice
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -fopenmp
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcarmodel/
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol/
IFLAGS += -I$(CARMEN_HOME)/sharedlib/Catch

LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -fopenmp

LFLAGS += -lglobal -lgeometry -lparam_interface -lipc -llocalize_ackerman_interface -lmap_interface -lrobot_ackerman_interface \
		-lmap_server_interface -lconventional_ackerman -lbase_ackerman_interface -lbehavior_selector_interface -lgrid_mapping \
		-lmap_io -lmap_util -lmap_interface -lmapper_interface -lprob_models  -llocalize_ackerman_interface -lrotation_geometry \
		-lcarmodel -lm -lsimulator_ackerman_interface -lobstacle_distance_mapper_interface
		
CFLAGS += -std=c99 # -O4

MODULE_NAME = OBSTACLE_AVOIDER_DAEMON
MODULE_COMMENT = Module integrating all robot sensors and configuration

SOURCES = obstacle_avoider.c obstacle_avoider_interface.c \
	obstacle_avoider_main.c collision_detection.c obstacle_avoider_test.c 
	#collision_detection_test.cpp
	
PUBLIC_INCLUDES = obstacle_avoider_interface.h obstacle_avoider_messages.h collision_detection.h
PUBLIC_LIBRARIES = libobstacle_avoider_interface.a libobstacle_avoider.a libcollision_detection.a
PUBLIC_BINARIES = obstacle_avoider obstacle_avoider_test

TARGETS = libcarmodel libobstacle_avoider.a obstacle_avoider libobstacle_avoider_interface.a libcollision_detection.a obstacle_avoider_test #collision_detection_test

# If you set graphics support in configure.
ifndef NO_GRAPHICS
#Carmen Global Graphic Library and Gtk+2 Graphics Libraries
LFLAGS += -lglobal_graphics
IFLAGS += `pkg-config --cflags gtk+-2.0 gmodule-export-2.0`
IFLAGS += `pkg-config --cflags gtkglext-1.0`
LFLAGS += `pkg-config --libs gtk+-2.0 gmodule-export-2.0`
LFLAGS += `pkg-config --libs gtkglext-1.0`
endif

PUBLIC_LIBRARIES_SO = libobstacle_avoider_interface.so

libcarmodel:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libcarmodel
clean: clean_libcarmodel
clean_libcarmodel:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libcarmodel

obstacle_avoider:obstacle_avoider.o libobstacle_avoider.a libobstacle_avoider_interface.a libcollision_detection.a

libobstacle_avoider.a: obstacle_avoider_main.o 

libobstacle_avoider_interface.a:obstacle_avoider_interface.o

libcollision_detection.a: collision_detection.o

obstacle_avoider_test: obstacle_avoider_test.o

#collision_detection_test: collision_detection_test.o collision_detection.o

include ../Makefile.rules
