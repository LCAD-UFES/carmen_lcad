include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = SEG MAP UTIL TESTS
MODULE_COMMENT = Tests programs of the libsegmap_util library 

CXXFLAGS += -Wno-sign-compare -std=c++11
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11

LFLAGS += -lsegmap_util -lboost_program_options

SOURCES = test_libsegmap_util.cpp
TARGETS = test_libsegmap_util

test_libsegmap_util: test_libsegmap_util.o

include $(CARMEN_HOME)/src/Makefile.rules
