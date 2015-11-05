include ../Makefile.conf

# voice commands
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libvoice
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libvoice -lvoice -lespeak
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models

LFLAGS += -lglobal -lgeometry -lparam_interface -lipc \
		-llocalize_ackerman_interface -lmap_interface -lrobot_ackerman_interface \
		-lgrid_mapping_interface -lmap_server_interface -lconventional_ackerman \
		-lbehavior_selector_interface -lgrid_mapping -lmap_io -lmap_util -lmap_interface -lgrid_mapping_interface \
		-lprob_models  -llocalize_ackerman_interface -lrotation_geometry
		
CFLAGS += -std=c99 # -O4

MODULE_NAME = OBSTACLE_AVOIDER_DAEMON
MODULE_COMMENT = Module integrating all robot sensors and configuration

SOURCES = obstacle_avoider.c obstacle_avoider_interface.c \
	obstacle_avoider_main.c collision_detection.c
	
PUBLIC_INCLUDES = obstacle_avoider_interface.h obstacle_avoider_messages.h collision_detection.h
PUBLIC_LIBRARIES = libobstacle_avoider_interface.a libobstacle_avoider.a libcollision_detection.a
PUBLIC_BINARIES = obstacle_avoider 

TARGETS = libvoice.a libobstacle_avoider.a obstacle_avoider libobstacle_avoider_interface.a libcollision_detection.a

PUBLIC_LIBRARIES_SO = libobstacle_avoider_interface.so

libvoice.a:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libvoice
	
obstacle_avoider:obstacle_avoider.o libobstacle_avoider.a libobstacle_avoider_interface.a libcollision_detection.a

libobstacle_avoider.a: obstacle_avoider_main.o 

libobstacle_avoider_interface.a:obstacle_avoider_interface.o

libcollision_detection.a: collision_detection.o

include ../Makefile.rules
