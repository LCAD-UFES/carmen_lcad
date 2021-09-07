include ../../src/Makefile.conf

LINK = g++
CFLAGS += -fopenmp -std=c11
CXXFLAGS += -fopenmp -std=c++11

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet -I/usr/include/bullet
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src 

LFLAGS += -lz -lmap_io -lmap_interface -lmap_util -lglobal -lipc -llocalize_ackerman_core  \
	  -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt \
	  -lrt -lboost_signals -lboost_system

MODULE_NAME = PROB_MODELS
MODULE_COMMENT = Probabilistic Models LIB 

SOURCES = prob_map.cpp prob_measurement_model.cpp prob_motion_model.cpp prob_monte_carlo.cpp prob_interface.c prob_transforms.cpp
PUBLIC_INCLUDES = prob_map.h prob_measurement_model.h
#PUBLIC_INCLUDES = prob_map.h prob_measurement_model.h prob_motion_model.h prob_monte_carlo.h prob_interface.h prob_messages.h prob_transforms.h \
	prob_types.h

#PUBLIC_LIBRARIES = libprob_models.a

TARGETS = libprob_models.a

ifdef CARMEN_DEBUG
CFLAGS += -DDEBUG
endif

libprob_models.a: prob_map.o prob_measurement_model.o prob_motion_model.o prob_monte_carlo.o prob_interface.o prob_transforms.o

include ../../src/Makefile.rules

