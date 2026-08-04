include ../Makefile.conf

CFLAGS += -DOLD_MOTION_MODEL -std=gnu99

PYTHON_CONFIG ?= /usr/bin/python3-config
OPENJAUS_BASE := $(CARMEN_HOME)/sharedlib/OpenJAUS
OPENJAUS_LIB_DIRS := -L$(OPENJAUS_BASE)/libjaus/lib -L$(OPENJAUS_BASE)/libopenJaus/lib -L$(OPENJAUS_BASE)/ojTorc/lib
OPENJAUS_RPATH := -Wl,-rpath,$(OPENJAUS_BASE)/libjaus/lib:$(OPENJAUS_BASE)/libopenJaus/lib:$(OPENJAUS_BASE)/ojTorc/lib

IFLAGS += -I../obstacle_avoider
IFLAGS += -I$(CARMEN_HOME)/sharedlib/OpenJAUS/ojTorc/include/
IFLAGS += -I$(CARMEN_HOME)/sharedlib/OpenJAUS/libjaus/include/
IFLAGS += -I$(CARMEN_HOME)/sharedlib/OpenJAUS/libopenJaus/include/
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcarmodel/
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol/
#IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol/mpc
IFLAGS += -I$(CARMEN_HOME)/include/carmen/
IFLAGS += `$(PYTHON_CONFIG) --includes`

LFLAGS += -lmap_interface -lparam_interface -llocalize_ackerman_interface -lobstacle_avoider -lobstacle_avoider_interface \
	  -llaser_interface -lgeometry -llocalize_ackerman_motion -lmotion_planner_interface -lbase_ackerman_interface -lvisual_odometry_interface \
	  -lglobal -lipc -lmapper_interface -lfused_odometry_interface -lcollision_detection -lmap_server_interface -lbehavior_selector_interface \
	  -ltask_manager_interface -lopenJaus -ljaus -lojTorc -lcontrol -lcarmodel -lfann -lgsl -lgslcblas -lford_escape_hybrid_interface \
	  -lrobot_ackerman_interface -lvoice_interface_interface -lsimulator_ackerman -lsimulator_ackerman_interface

LFLAGS += $(OPENJAUS_LIB_DIRS) $(OPENJAUS_RPATH)
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcarmodel/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcontrol/
#LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcontrol/mpc


LFLAGS += `$(PYTHON_CONFIG) --embed --libs` -pthread

MODULE_NAME = FORD_ESCAPE_HYBRID
MODULE_COMMENT = "LCAD driver for TORC/JAUS Ford Escape Hybrid"

SOURCES = ford_escape_hybrid.c ford_escape_hybrid_interface.c ford_escape_hybrid_tune_pid.c ford_escape_hybrid_tune_pid_automatic.cpp \
ford_escape_hybrid_test.c ford_escape_hybrid_test_curves.c fake_odometry.c

PUBLIC_INCLUDES = ford_escape_hybrid_messages.h ford_escape_hybrid_interface.h ford_escape_hybrid.h
PUBLIC_LIBRARIES = libford_escape_hybrid_interface.a
PUBLIC_BINARIES = ford_escape_hybrid ford_escape_hybrid_tune_pid ford_escape_hybrid_test ford_escape_hybrid_test_curves fake_odometry

TARGETS = OpenJAUS libcontrol libcarmodel ford_escape_hybrid ford_escape_hybrid_tune_pid \
		  fake_odometry libford_escape_hybrid_interface.a ford_escape_hybrid_test ford_escape_hybrid_test_curves ford_escape_hybrid_tune_pid_automatic

ifndef NO_PYTHON
PUBLIC_LIBRARIES_SO = libford_escape_hybrid_interface.so
TARGETS += libford_escape_hybrid_interface.so.1
endif

# rules
OpenJAUS:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/OpenJAUS/

libcarmodel:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libcarmodel
clean: clean_libcarmodel
clean_libcarmodel:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libcarmodel

libcontrol:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libcontrol
clean: clean_libcontrol
clean_libcontrol:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libcontrol

ford_escape_hybrid: ford_escape_hybrid.o

ford_escape_hybrid_tune_pid: ford_escape_hybrid_tune_pid.o

ford_escape_hybrid_test: ford_escape_hybrid_test.o

ford_escape_hybrid_test_curves: ford_escape_hybrid_test_curves.o

fake_odometry: fake_odometry.o

libford_escape_hybrid_interface.a: ford_escape_hybrid_interface.o

libford_escape_hybrid_interface.so.1: ford_escape_hybrid_interface.o

ford_escape_hybrid_tune_pid_automatic: ford_escape_hybrid_tune_pid_automatic.o

include ../Makefile.rules
