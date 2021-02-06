
include ../Makefile.conf

MODULE_NAME = Localize Neural 2.0
MODULE_COMMENT = Classifies the images of the environment and delivers the location.

LINK = g++ -g
CXXFLAGS = -std=c++11 -g

IFLAGS += -I$(CARMEN_HOME)/sharedlib/darknet4/src -I$(CARMEN_HOME)/sharedlib/darknet4/include 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet4/


LFLAGS += -fPIC -l:libdarknet.so -lstdc++ -g

SOURCES = localize_neural2.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = localize_neural2

TARGETS = darknet localize_neural2

localize_neural2: localize_neural2.o

darknet:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/darknet4  > /dev/null 2>&1

clean: clean_darknet clean_localize_neural2

clean_darknet:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/darknet4

clean_localize_neural2:
	rm -f $(TARGETS)
	
include ../Makefile.rules




