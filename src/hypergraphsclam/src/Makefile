include ../../Makefile.conf

LINK = g++

MODULE_NAME = hypergraphsclam Grab Data
MODULE_COMMENT =

CXXFLAGS += -std=c++11

IFLAGS += -I$(CARMEN_HOME)/src/hypergraphsclam/src \
			-I$(CARMEN_HOME)/src/hypergraphsclam/Helpers \
			-I$(CARMEN_HOME)/src/hypergraphsclam/Messages \
			-I$(CARMEN_HOME)/src/hypergraphsclam/CustomEdges \
			-I$(CARMEN_HOME)/src/hypergraphsclam/src \

#cheating
IFLAGS += -std=c++11

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

###IFLAGS += -I/usr/include/suitesparse -I/usr/include/eigen3 -I/usr/include/pcl-1.7  -I/usr/include/vtk-5.8

SOURCES = VehicleModel.cpp GrabData.cpp

include ../../Makefile.rules
