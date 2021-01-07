
include ../Makefile.conf

MODULE_NAME = Localize Neural 2.0
MODULE_COMMENT = Classifies the images of the environment and delivers the location.

LINK = g++
CXXFLAGS = -std=c++11

IFLAGS += -I$(CARMEN_HOME)/sharedlib/darknet4/src -I$(CARMEN_HOME)/sharedlib/darknet4/include 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet4/


LFLAGS += -fPIC -l:libdarknet.so -lstdc++

SOURCES = classifier.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = config/classifier

TARGETS = darknet classifier

classifier: classifier.o

darknet:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/darknet4

clean: clean_darknet clean_classifier

clean_darknet:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/darknet4

clean_classifier:
	rm -f $(PUBLIC_BINARIES)
	
include ../Makefile.rules




