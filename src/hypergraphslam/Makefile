include ../Makefile.conf

MODULE_NAME = HYPERGRAPHSCLAM_PACKAGE
MODULE_COMMENT = Sensor Calibration

LINK = g++

CXXFLAGS += -std=c++11 -O3

#cheating
IFLAGS += -std=c++11 -O3

IFLAGS += 	-I$(CARMEN_HOME)/src/hypergraphslam \
			-I$(CARMEN_HOME)/src/hypergraphslam/Helpers \
			-I$(CARMEN_HOME)/src/hypergraphslam/Messages \
			-I$(CARMEN_HOME)/src/hypergraphslam/CustomEdges \
			-I$(CARMEN_HOME)/src/hypergraphslam/src

# the eigen includes
IFLAGS += -I/usr/include/eigen3

# PCL includes
IFLAGS += -I/usr/include/pcl-1.7/

# the suitesparse includes
IFLAGS += -I/usr/include/suitesparse

# the g2o includes
IFLAGS += -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/

#boost
LFLAGS += -lboost_system

# PCL
LFLAGS += -lpcl_common -lpcl_io -lpcl_registration -lpcl_features -lpcl_search -lpcl_filters -lpcl_visualization

# g2o libs
LFLAGS += -L/usr/local/lib -lcholmod -lg2o_core -lg2o_types_slam2d -lg2o_solver_cholmod -lg2o_stuff

# the carmen libraries
LFLAGS += -lglobal

# the messages library
#LFLAGS += -L$(CARMEN_HOME)/src/hypergraphslam/Messages/

SUBDIRS = Helpers/ Messages/ src/ CustomEdges/

SOURCES = 	Helpers/StringHelper.cpp Messages/StampedOdometry.cpp \
			Messages/StampedGPSPose.cpp \
			Messages/StampedGPSOrientation.cpp \
			Messages/StampedXSENS.cpp \
			Messages/StampedLidar.cpp \
			Messages/StampedSICK.cpp \
			Messages/StampedVelodyne.cpp \
			src/VehicleModel.cpp \
			src/GrabData.cpp \
			src/HyperGraphSclamOptimizer.cpp \
			parser.cpp \
			hypergraphslam.cpp

#PUBLIC_INCLUDES =
#PUBLIC_LIBRARIES =
#PUBLIC_BINARIES = hypergraphslam parser

TARGETS = hypergraphslam parser

hypergraphslam: src/VehicleModel.o Helpers/StringHelper.o src/HyperGraphSclamOptimizer.o hypergraphslam.o

parser:	Helpers/StringHelper.o Messages/StampedOdometry.o \
				Messages/StampedGPSPose.o \
				Messages/StampedGPSOrientation.o \
				Messages/StampedXSENS.o  \
				Messages/StampedLidar.o \
				Messages/StampedSICK.o \
				Messages/StampedVelodyne.o \
				src/VehicleModel.o \
				src/GrabData.o \
				parser.o

include ../Makefile.rules
