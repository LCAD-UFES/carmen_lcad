include ../../Makefile.conf

LINK = g++

MODULE_NAME = HYPERGRAPHSCLAM HELPERS
MODULE_COMMENT =

CXXFLAGS += -std=c++11 -O2

IFLAGS += -I$(CARMEN_HOME)/src/hypergraphsclam/Helpers/ -std=c++11 -O2

# the eigen includes
IFLAGS += -I/usr/include/eigen3

# PCL includes
IFLAGS += -I/usr/include/pcl-1.7/

SOURCES = StringHelper.cpp SimpleLidarSegmentation.cpp

include ../../Makefile.rules