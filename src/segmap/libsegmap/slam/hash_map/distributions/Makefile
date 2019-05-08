
include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = LIB_SEG_MAP HASH MAP DISTRIBUTIONS
MODULE_COMMENT = Probability Distributions

LINK = g++ 

CXXFLAGS += -Wno-deprecated -Wno-write-strings -Wno-unused-result -Wno-sign-compare -std=c++11 
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11

SOURCES = segmap_bernoulli.cpp segmap_categorical.cpp segmap_gaussian.cpp
PUBLIC_INCLUDES = segmap_distribution_interface.h segmap_bernoulli.h segmap_categorical.h segmap_gaussian.h
PUBLIC_LIBRARIES = libsegmap_distributions.a 
PUBLIC_BINARIES = 
TARGETS = libsegmap_distributions.a 

libsegmap_distributions.a: segmap_bernoulli.o segmap_categorical.o segmap_gaussian.o

include $(CARMEN_HOME)/src/Makefile.rules
