include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CXXFLAGS += -std=c++0x

MODULE_NAME = CAR MODEL
MODULE_COMMENT = Car Neural and Ackeman Motion Model

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcontrol

LFLAGS += -lparam_interface -lipc -lglobal -lfann -ltask_manager_interface -lgsl -lgslcblas

SOURCES = neural_model.cpp ackerman_model.cpp # evaluate_car_model.cpp
 
PUBLIC_INCLUDES = car_model.h
PUBLIC_LIBRARIES = libcarmodel.a
#PUBLIC_BINARIES =

TARGETS = libcarmodel.a # evaluate_car_model
PUBLIC_LIBRARIES_SO =

libcarmodel.a: neural_model.o ackerman_model.o

#evaluate_car_model: evaluate_car_model.o libcarmodel.a

include $(CARMEN_HOME)/src/Makefile.rules
