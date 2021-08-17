include ../Makefile.conf

MODULE_NAME = Localize Neural 2.0
MODULE_COMMENT = Classifies the images of the environment and delivers the location.

LINK = g++ -g
CXXFLAGS = -std=c++11 -g

IFLAGS += -I$(CARMEN_HOME)/sharedlib/darknet4/src -I$(CARMEN_HOME)/sharedlib/darknet4/include 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet4/

CFLAGS += `pkg-config --cflags opencv` -I/usr/include/gflags

LFLAGS += -lparam_interface -lipc -lglobal -lbumblebee_basic_interface -lgps_xyz_interface -lcamera_drivers -lvelodyne_interface
LFLAGS += -fPIC -lgflags -ldarknet4 -lstdc++ `pkg-config --libs opencv` 

SOURCES = deep_vgl-test.cpp deep_vgl-eval.cpp deep_vgl_main.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = deep_vgl-test deep_vgl-eval deep_vgl

TARGETS = darknet deep_vgl-test deep_vgl-eval deep_vgl

deep_vgl-test: deep_vgl-test.o

deep_vgl-eval: deep_vgl-eval.o

deep_vgl: deep_vgl_main.o

darknet:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/darknet4  > /dev/null 2>&1

clean: clean_localize_neural2

clean_darknet:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/darknet4

clean_localize_neural2:
	rm -f $(TARGETS)
	
include ../Makefile.rules




