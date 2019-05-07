include ../../Makefile.conf

LINK = g++

MODULE_NAME = HYPERGRAPHSCLAM GRAB DATA
MODULE_COMMENT =

CXXFLAGS += -std=c++11 -O3 -Wno-deprecated-declarations

IFLAGS += -I$(CARMEN_HOME)/src/hypergraphsclam/src \
			-I$(CARMEN_HOME)/src/hypergraphsclam/Helpers \
			-I$(CARMEN_HOME)/src/hypergraphsclam/Messages \
			-I$(CARMEN_HOME)/src/hypergraphsclam/CustomEdges \
			-I$(CARMEN_HOME)/src/hypergraphsclam/src \

#cheating
IFLAGS += -std=c++11 -O3

IFLAGS += -I/usr/include/eigen3 -I /usr/local/include/pcl-1.8

# the eigen includes
IFLAGS += -I/usr/include/eigen3

# the suitesparse includes
IFLAGS += -I/usr/include/suitesparse

# the g2o includes
IFLAGS += -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/

# the pcl includes
IFLAGS += -I/usr/include/pcl-1.7

# the carmen libs
LFLAGS += -lglobal

# the libviso includes
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libviso2.3/src

SOURCES = VehicleModel.cpp GrabData.cpp HyperGraphSclamOptimizer.cpp

include ../../Makefile.rules
