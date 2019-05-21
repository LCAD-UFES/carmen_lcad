include ../Makefile.conf

MODULE_NAME = Neural Object Detector Fovea
MODULE_COMMENT = Detect vehicles using convolutional neural networks.

LINK = g++
CXXFLAGS = -std=c++11

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I/usr/include/python2.7 -DBOOST_SIGNALS_NO_DEPRECATION_WARNING 

LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet2/lib -L$(CARMEN_HOME)/sharedlib/libtf/src

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libsickldmrs2/include
# Required default libraries to comunicate with Carmen Core.
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libsickldmrs2 

LFLAGS += -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lparam_interface -llocalize_ackerman_interface \
		  -lmoving_objects_interface -ldarknet2 `pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics \
		  -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system -lvelodyne_camera_calibration \
		  -llaser_ldmrs_interface -llaser_ldmrs_utils `python2.7-config --libs`

SOURCES = neural_object_detector_main.cpp neural_object_detector_simulator.cpp neural_object_detector_tracker.cpp

PUBLIC_BINARIES = neural_object_detector neural_object_detector_simulator neural_object_detector_tracker

TARGETS = darknet2 neural_object_detector neural_object_detector_simulator neural_object_detector_tracker

neural_object_detector: neural_object_detector_main.o

neural_object_detector_simulator: neural_object_detector_simulator.o

neural_object_detector_tracker: neural_object_detector_tracker.o

darknet2:
	#$(MAKE) -C $(CARMEN_HOME)/sharedlib/darknet2


include ../Makefile.rules
