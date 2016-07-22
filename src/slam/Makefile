include ../Makefile.conf

LINK=g++
CFLAGS += -std=c99

IFLAGS += -I$(CARMEN_HOME)/sharedlib/gmapping -I$(CARMEN_HOME)/sharedlib/gmapping/sensor
LFLAGS += -L$(CARMEN_HOME)/sharedlib/gmapping/lib

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src

LFLAGS += -lglobal -lparam_interface -lipc -lprob_models \
	-llocalize_interface -llocalize_ackerman_interface -lmapper_interface \
	-lbase_interface -lmap_interface -lrobot_ackerman_interface \
	-lobstacle_avoider_interface -lsimulator_ackerman_interface \
	-ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system \
	-lrotation_geometry -lvelodyne_interface -lfused_odometry_interface

MODULE_NAME = SLAM
MODULE_COMMENT = Monte Carlo FastSLAM2

SOURCES = slam_gmapping.cpp slam_montecarlo.c
PUBLIC_INCLUDES = slam_messages.h 

PUBLIC_BINARIES = slam_mc
MAN_PAGES =

TARGETS = libprob_models.a slam_mc 

PUBLIC_LIBRARIES_SO = 

ifndef NO_PYTHON
TARGETS += 
endif

ifdef CARMEN_DEBUG
CFLAGS += -DDEBUG
endif

slam: slam_gmapping.o
 
slam_mc: slam_montecarlo.o

# rules

libprob_models.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/prob_models/

clean: clean_prob_models

clean_prob_models:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/prob_models/
	
include ../Makefile.rules

