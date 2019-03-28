include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = LIB_SEG_MAP MOTION MODELS
MODULE_COMMENT = Library of motion models

LINK = g++ 

CXXFLAGS += -Wno-deprecated -Wno-write-strings -Wno-unused-result -Wno-sign-compare -std=c++11 
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11

IFLAGS += -I /usr/include/eigen3/ 

SOURCES = ackerman_motion_model.cpp

PUBLIC_INCLUDES = ackerman_motion_model.h
PUBLIC_LIBRARIES = libsegmap_motion_models.a
PUBLIC_BINARIES = 
TARGETS = libsegmap_motion_models.a

libsegmap_motion_models.a: ackerman_motion_model.o

include $(CARMEN_HOME)/src/Makefile.rules
