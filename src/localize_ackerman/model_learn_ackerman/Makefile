include ../../Makefile.conf

SUBDIRS = learn_model_ackerman

##CC = $(CXX)
LINK = $(CXX)

CFLAGS += 
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -I$(CARMEN_HOME)/sharedlib/gl_plotter 
LFLAGS += -lreadlog -lkinect_interface -lglobal -lparam_interface -lipc `pkg-config --libs opencv` -pthread

MODULE_NAME = MODEL-LEARN ACKERMAN
MODULE_COMMENT = A program for learning motion models.

SOURCES = mt-rand_ackerman.cpp basic_ackerman.cpp map_ackerman.cpp lowMap_ackerman.cpp low_ackerman.cpp learn_ackerman.cpp slam_ackerman.cpp 
PUBLIC_INCLUDES = l
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = model_learner_ackerman
MAN_PAGES =

TARGETS = model_learner_ackerman

# rules

model_learner_ackerman: mt-rand_ackerman.o basic_ackerman.o map_ackerman.o lowMap_ackerman.o low_ackerman.o learn_ackerman.o \
	slam_ackerman.o 

include ../../Makefile.rules



