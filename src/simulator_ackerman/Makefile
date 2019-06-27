include ../Makefile.conf

MODULE_NAME = SIMULATOR ACKERMAN
MODULE_COMMENT = "simulates the readings of a ackerman robot on a map"

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

SOURCES = simulator_ackerman.c objects_ackerman.c simulator_ackerman_simulation.c simulator_ackerman_test.c \
	simulator_ackerman_interface.c

PUBLIC_INCLUDES = simulator_ackerman_messages.h simulator_ackerman_interface.h simulator_ackerman.h \
	simulator_ackerman_simulation.h objects_ackerman.h
PUBLIC_LIBRARIES = libsimulator_ackerman_interface.a libsimulator_ackerman.a
PUBLIC_BINARIES = simulator_ackerman simulator_ackerman_connect_multiple
MAN_PAGES =

TARGETS = libcarmodel libcontrol simulator_ackerman simulator_ackerman_test  libsimulator_ackerman_interface.a \
	simulator_ackerman_connect_multiple libsimulator_ackerman.a


ifndef NO_PYTHON
PUBLIC_LIBRARIES_SO = libsimulator_ackerman_interface.so
TARGETS += libsimulator_ackerman_interface.so.1
endif


libcarmodel:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libcarmodel
	
clean: clean_libcarmodel

clean_libcarmodel:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libcarmodel


libcontrol:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libcontrol/
	
clean: clean_libcontrol

clean_libcontrol:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libcontrol/


libsimulator_ackerman.a: simulator_ackerman.o simulator_ackerman_simulation.o objects_ackerman.o

simulator_ackerman:	simulator_ackerman.o simulator_ackerman_simulation.o objects_ackerman.o

libsimulator_ackerman_interface.a: simulator_ackerman_interface.o

libsimulator_ackerman_interface.so.1: simulator_ackerman_interface.o

simulator_ackerman_test: simulator_ackerman_test.o

simulator_ackerman_connect_multiple: simulator_ackerman_connect_multiple.o simulator_ackerman_interface.o

tst_ackerman : tst_ackerman.o simulator_ackerman_graphics.o simulator_ackerman_simulation.o objects_ackerman.o

include ../Makefile.rules
