include ../Makefile.conf

LINK = g++

CFLAGS += -DOLD_MOTION_MODEL -std=c99

IFLAGS += -I../obstacle_avoider
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcarmodel
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol
IFLAGS += -I$(CARMEN_HOME)/include/carmen

LFLAGS += -lmap_interface -lparam_interface -llocalize_ackerman_interface -lobstacle_avoider \
	  -llaser_interface -lgeometry -llocalize_ackerman_motion -lbase_ackerman_interface \
	  -lglobal -lipc -lfused_odometry_interface -lcollision_detection -lmap_server_interface -lmapper_interface \
	  -lmap_server_interface -lford_escape_hybrid_interface -lcontrol -lcarmodel -lfann -lgsl -lgslcblas \
	  -lobstacle_avoider_interface

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcarmodel
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcontrol

MODULE_NAME = SIMULATOR ACKERMAN
MODULE_COMMENT = "simulates the readings of a ackerman robot on a map"

SOURCES = simulator_ackerman.c objects_ackerman.c simulator_ackerman_simulation.c simulator_ackerman_test.c \
	simulator_ackerman_interface.c


PUBLIC_INCLUDES = simulator_ackerman_messages.h simulator_ackerman_interface.h simulator_ackerman.h
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = simulator_ackerman2
MAN_PAGES =


TARGETS = simulator_ackerman2


simulator_ackerman2: simulator_ackerman.o simulator_ackerman_simulation.o objects_ackerman.o


include ../Makefile.rules
