include ../Makefile.conf

MODULE_NAME = Deep Mapper 
MODULE_COMMENT = Estimage Depth Maps from monocular image cameras.

LINK = g++ -g
CXXFLAGS = -std=c++11 -g
IFLAGS += -I/usr/include/python3.6
# IFLAGS += -I$(CARMEN_HOME)/sharedlib/darknet4/src -I$(CARMEN_HOME)/sharedlib/darknet4/include 
# LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet4/

CFLAGS += `pkg-config --cflags opencv` -I/usr/include/gflags `python3.6m-config --cflags`

LFLAGS += -lparam_interface -lipc -lglobal -lbumblebee_basic_interface  -lcamera_drivers
LFLAGS += -fPIC -lgflags -lstdc++ `pkg-config --libs opencv` `python3.6m-config --ldflags --libs` 

SOURCES = deep_mapper_main.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = deep_mapper

TARGETS = deep_mapper

deep_mapper: deep_mapper_main.o

clean: clean_deep_mapper

clean_deep_mapper:
	rm -f $(TARGETS)
	
include ../Makefile.rules




