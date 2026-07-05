include ../../src/Makefile.conf

LINK ?= g++
CXXFLAGS += -std=c++11 -D_GLIBCXX_USE_CXX11_ABI=0 -Wno-deprecated-copy

MODULE_NAME = LIBTRAILER_NLP
MODULE_COMMENT = Tractor-trailers Trajectory Planning Library

IFLAGS += -I$(CARMEN_HOME)/include/carmen

#G2O
IFLAGS += -I/usr/include/eigen3 
IFLAGS += -I/usr/include/suitesparse -I/usr/include/eigen3 -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/

#CASADI
CASADI_DIR=$(shell /usr/bin/env python3 -c "import casadi; print(casadi.__path__[0])")
CXXFLAGS += -Wno-sign-compare -Wno-unused-parameter 
IFLAGS += -I$(CASADI_DIR)/include
LFLAGS += -L$(CASADI_DIR) -lcasadi -lstdc++ -lglobal_planning_interface -L/usr/lib64/libkml -lkmlbase -lkmldom -lkmlengine -lcarmen_gps_wrapper -ltask_manager_interface -lcarmen_util


SOURCES = trailer_nlp.cpp

PUBLIC_INCLUDES = trailer_nlp.h 

PUBLIC_LIBRARIES = libtrailer_nlp.a

TARGETS = libtrailer_nlp.a

libtrailer_nlp.a: trailer_nlp.o

trailer_nlp.o: trailer_nlp.h

include ../../src/Makefile.rules