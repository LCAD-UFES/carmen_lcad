include ../Makefile.conf

MODULE_NAME = FUSED ODOMETRY
MODULE_COMMENT = This module implements a particle filter that uses IMU, GPS, and odometry data to produce a fused pose.

LINK = g++
CXXFLAGS += -Wall -msse3 -Wno-parentheses -Wno-deprecated-copy -Wno-implicit-fallthrough -Wno-deprecated-declarations

CFLAGS += -std=c11
CXXFLAGS += -std=c++11

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models
IFLAGS += -I/usr/include/boost
LFLAGS += -I/usr/lib/x86_64-linux-gnu
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I/usr/include/bullet/ -I/usr/include/opencv4/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L/usr/local/carmen_boost/lib
LFLAGS += -lglobal -lipc -lparam_interface -lxsens_interface -lgps_xyz_interface
LFLAGS += -llocalize_ackerman_interface -llocalize_ackerman_motion -lvisual_odometry_interface \
		  -lxsens_mtig_interface -lbase_ackerman_interface -lgps_nmea_interface \
		  -lrotation_geometry -ltf -lBulletCollision -lBulletDynamics \
		  -lBulletSoftBody -lLinearMath -lboost_thread -lrt -lboost_system -lm -pthread

IFLAGS += `pkg-config --cflags opencv4`
LFLAGS += `pkg-config --libs opencv4`

SOURCES = fused_odometry_kalman.cpp fused_odometry_main.cpp fused_odometry.cpp fused_odometry_interface.c xsens_xyz_handler.cpp visual_odometry_handler.cpp rotation_geometry.c tf_test.cpp \
	  car_odometry_handler.cpp
PUBLIC_INCLUDES = fused_odometry_messages.h fused_odometry_interface.h rotation_geometry.h

PUBLIC_LIBRARIES = libfused_odometry.a libfused_odometry_interface.a librotation_geometry.a
PUBLIC_BINARIES = fused_odometry
PUBLIC_LIBRARIES_SO = libfused_odometry_interface.so

TARGETS = libtf.a librotation_geometry.a libfused_odometry_interface.a fused_odometry tf_test

ifndef NO_PYTHON
TARGETS += libfused_odometry_interface.so.1
endif

# rules

librotation_geometry.a:	rotation_geometry.o

libfused_odometry.a:	fused_odometry.o

libfused_odometry_interface.a:	fused_odometry_interface.o

libfused_odometry_interface.so.1:	fused_odometry_interface.o

fused_odometry:	fused_odometry_main.o fused_odometry_kalman.o fused_odometry.o xsens_xyz_handler.o visual_odometry_handler.o car_odometry_handler.o libfused_odometry_interface.a librotation_geometry.a

tf_test: tf_test.o

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



