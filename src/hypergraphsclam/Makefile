include ../Makefile.conf

MODULE_NAME = HYPERGRAPHSCLAM PACKAGE
MODULE_COMMENT = Sensor Calibration

LINK = g++

CXXFLAGS += -std=c++11 -O3 -Wno-deprecated-declarations

#cheating
IFLAGS += -std=c++11 -O3

IFLAGS += -I$(CARMEN_HOME)/src/hypergraphsclam \
	-I$(CARMEN_HOME)/src/hypergraphsclam/Helpers \
	-I$(CARMEN_HOME)/src/hypergraphsclam/Messages \
	-I$(CARMEN_HOME)/src/hypergraphsclam/CustomElements \
	-I$(CARMEN_HOME)/src/hypergraphsclam/src

IFLAGS += -I/usr/include/

# eigen and PCL includes
PCL_INC = $(wildcard /usr/local/include/pcl-1.8*)

IFLAGS += -I/usr/include/eigen3 -I $(PCL_INC)

# the suitesparse includes
IFLAGS += -I/usr/include/suitesparse

# the g2o includes
IFLAGS += -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/

# the libviso includes
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libviso2.3/src

#boost
LFLAGS += -lboost_system -lboost_filesystem

# PCL
LFLAGS += -lpcl_common -lpcl_io -lpcl_registration -lpcl_features -lpcl_search -lpcl_filters -lpcl_visualization -lpcl_kdtree

# g2o libs
LFLAGS += -L/usr/local/lib -lcholmod -lg2o_core -lg2o_types_slam2d -lg2o_solver_cholmod -lg2o_solver_csparse -lg2o_stuff

# g2o libs...
LFLAGS += -lcxsparse -lg2o_csparse_extension 

UBUNTU_VERSION ="$(shell cat /etc/issue | grep Ubuntu | cut -c8-12)"
#$(warning now we reached ifeq UBUNTU_VERSION=$(UBUNTU_VERSION))
ifeq ($(UBUNTU_VERSION),"16.04")
	LFLAGS += -lcsparse
endif

# the carmen libraries
LFLAGS += -lglobal -lipc

# the png files
LFLAGS += -lpng

# the libviso sources
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libviso2.3/src -lviso

# the messages library
#LFLAGS += -L$(CARMEN_HOME)/src/hypergraphsclam/Messages/

# Profile flags
#CXXFLAGS += -pg
#LFLAGS += -pg

SUBDIRS = Helpers/ Messages/ src/ CustomElements/

SOURCES = Helpers/StringHelper.cpp \
	Helpers/SimpleLidarSegmentation.cpp \
	Messages/StampedOdometry.cpp \
	Messages/StampedGPSPose.cpp \
	Messages/StampedGPSOrientation.cpp \
	Messages/StampedXSENS.cpp \
	Messages/StampedLidar.cpp \
	Messages/StampedSICK.cpp \
	Messages/StampedVelodyne.cpp \
	Messages/StampedBumblebee.cpp \
	src/VehicleModel.cpp \
	src/GrabData.cpp \
	src/HyperGraphSclamOptimizer.cpp \
	parser.cpp \
	hypergraphsclam.cpp

TARGETS = libviso hypergraphsclam hypergraphsclam_parser

PUBLIC_BINARIES = hypergraphsclam hypergraphsclam_parser

libviso:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libviso2.3/src

hypergraphsclam: $(CARMEN_HOME)/sharedlib/libviso2.3/src/libviso.a src/VehicleModel.o Helpers/StringHelper.o src/HyperGraphSclamOptimizer.o hypergraphsclam.o

hypergraphsclam_parser:	Helpers/StringHelper.o \
	Helpers/SimpleLidarSegmentation.o \
	Messages/StampedOdometry.o \
	Messages/StampedGPSPose.o \
	Messages/StampedGPSOrientation.o \
	Messages/StampedXSENS.o  \
	Messages/StampedLidar.o \
	Messages/StampedSICK.o \
	Messages/StampedVelodyne.o \
	Messages/StampedBumblebee.o \
	src/VehicleModel.o \
	src/GrabData.o \
	parser.o

include ../Makefile.rules
