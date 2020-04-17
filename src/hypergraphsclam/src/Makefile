include ../../Makefile.conf

LINK = g++

MODULE_NAME = HYPERGRAPHSCLAM GRAB DATA
MODULE_COMMENT =

CXXFLAGS += -std=c++11 -O3 -Wno-deprecated-declarations

IFLAGS += -I$(CARMEN_HOME)/src/hypergraphsclam/src \
	-I$(CARMEN_HOME)/src/hypergraphsclam/Helpers \
	-I$(CARMEN_HOME)/src/hypergraphsclam/Messages \
	-I$(CARMEN_HOME)/src/hypergraphsclam/CustomElements \
	-I$(CARMEN_HOME)/src/hypergraphsclam/src \

#cheating
IFLAGS += -std=c++11 -O3

# the eigen and pcl includes
IFLAGS += -I/usr/include/eigen3 -I /usr/local/include/pcl-1.8

# the eigen includes
IFLAGS += -I/usr/include/eigen3

# the suitesparse includes
IFLAGS += -I/usr/include/suitesparse

# the g2o includes
IFLAGS += -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/

# the carmen libs
LFLAGS += -lglobal

# the libviso includes
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libviso2.3/src

# g2o libs
LFLAGS += -L/usr/local/lib -lcholmod -lg2o_core -lg2o_types_slam2d -lg2o_solver_cholmod -lg2o_solver_csparse -lg2o_stuff 

# g2o libs...
LFLAGS += -lcxsparse -lg2o_csparse_extension -lcsparse -lccholmod

SOURCES = VehicleModel.cpp GrabData.cpp HyperGraphSclamOptimizer.cpp

include ../../Makefile.rules
