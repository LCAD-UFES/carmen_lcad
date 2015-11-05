include ../Makefile.conf

MODULE_NAME = FUSED ODOMETRY 
MODULE_COMMENT = This module implements a Kalman filter that uses IMU, GPS, and odometry data to produce a fused pose.

LINK = g++
CXXFLAGS += -Wall -msse3

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -lglobal -lipc -lparam_interface -lxsens_interface -lgps_xyz_interface
LFLAGS += -lbase_ackerman_interface -llocalize_ackerman_interface -lvisual_odometry_interface -lxsens_mtig_interface
LFLAGS += -lrotation_geometry -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system -lm

IFLAGS += `pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv`

SOURCES = fused_odometry_kalman.cpp fused_odometry_main.cpp xsens_xyz_handler.cpp car_odometry_handler.cpp
PUBLIC_INCLUDES = 

PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = fused_odometry_kalman

TARGETS = fused_odometry_kalman libtf.a

ifndef NO_PYTHON
TARGETS += 
endif

# rules

fused_odometry_kalman:	fused_odometry_main.o fused_odometry_kalman.o xsens_xyz_handler.o car_odometry_handler.o

libtf.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libtf/src/

clean: clean_tf

clean_tf:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libtf/src/

libprob_models.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/prob_models/

clean: clean_prob_models

clean_prob_models:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/prob_models/
	
include ../Makefile.rules



