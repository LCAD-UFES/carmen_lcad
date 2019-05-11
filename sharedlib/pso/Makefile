include ../../src/Makefile.conf

LINK = g++
# ATENCAO! Para usar OpenMP a sua fitness fucntion precisa ser thread safe!
CFLAGS += -Wall -fopenmp
CXXFLAGS += -fopenmp
#CFLAGS += -Wall
#CXXFLAGS += 
IFLAGS +=
LFLAGS += -lglobal

MODULE_NAME = PSO LIB
MODULE_COMMENT = Particle Swarm Optimization Lib 

PUBLIC_INCLUDES = pso.h
PUBLIC_LIBRARIES = libpso++.a libpso.a

SOURCES = pso.cpp pso_c.c pso_test.cpp
TARGETS = libpso++.a pso_test pso_c

libpso++.a: pso.o
pso_test: pso_test.o pso.o 
pso_c: pso_c.o

libprob_models.a: prob_map.o prob_measurement_model.o prob_motion_model.o prob_monte_carlo.o prob_interface.o prob_transforms.o

include ../../src/Makefile.rules

