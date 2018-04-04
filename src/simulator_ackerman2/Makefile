include ../Makefile.conf

LINK = g++
CXXFLAGS += -std=c++0x

IFLAGS += -I../obstacle_avoider
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcarmodel
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol
IFLAGS += -I$(CARMEN_HOME)/include/carmen

LFLAGS += -lmap_interface -lparam_interface -llocalize_ackerman_interface -lobstacle_avoider \
	  -llaser_interface -lgeometry -llocalize_ackerman_motion -lbase_ackerman_interface \
	  -lglobal -lipc -lfused_odometry_interface -lcollision_detection -lmap_server_interface -lmapper_interface \
	  -lmap_server_interface -lford_escape_hybrid_interface -lcontrol -lcarmodel -lfann -lgsl -lgslcblas \
	  -lobstacle_avoider_interface -lsimulator_ackerman

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcarmodel
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcontrol

MODULE_NAME = SIMULATOR ACKERMAN2
MODULE_COMMENT = "simulates the readings of a ackerman robot on a map"

SOURCES = simulator_ackerman2.cpp simulator_ackerman2_simulation.cpp


PUBLIC_INCLUDES = simulator2_ackerman_simulation.h
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = simulator_ackerman2
MAN_PAGES =


TARGETS = simulator_ackerman2


simulator_ackerman2: simulator_ackerman2.o simulator_ackerman2_simulation.o


include ../Makefile.rules
