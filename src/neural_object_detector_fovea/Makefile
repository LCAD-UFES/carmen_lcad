include ../Makefile.conf

# Migração Ubuntu 26.04: o módulo linkava contra o OpenCV mas nunca pedia os -I dele.
# No Ubuntu 20.04 os headers ficavam em /usr/include/opencv2 (path padrão); o pacote do
# 26.04 põe tudo em /usr/include/opencv4, que exige -I explícito.
IFLAGS += `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv`

MODULE_NAME = Neural Object Detector Fovea
MODULE_COMMENT = Detect vehicles using convolutional neural networks.

LINK = g++
CXXFLAGS = -std=c++14

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ `pkg-config --cflags python3 2>/dev/null || python3-config --includes` -DBOOST_SIGNALS_NO_DEPRECATION_WARNING 

LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet2/lib -L$(CARMEN_HOME)/sharedlib/libtf/src

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libsickldmrs2/include
# Required default libraries to comunicate with Carmen Core.
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libsickldmrs2 

LFLAGS += -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lparam_interface -llocalize_ackerman_interface \
		  -lmoving_objects_interface -ldarknet2 `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics \
		  -lBulletSoftBody -lLinearMath -lboost_thread -lrt -lvelodyne_camera_calibration \
		  -llaser_ldmrs_interface -llaser_ldmrs_utils `pkg-config --libs python3-embed 2>/dev/null || python3-config --embed --libs`

SOURCES = neural_object_detector_simulator.cpp neural_object_detector_tracker.cpp neural_object_detector.cpp

PUBLIC_BINARIES = neural_object_detector_simulator neural_object_detector_tracker

TARGETS = darknet2 neural_object_detector_simulator neural_object_detector_tracker

neural_object_detector_simulator: neural_object_detector_simulator.o

neural_object_detector_tracker: neural_object_detector_tracker.o neural_object_detector.o

darknet2:
	#$(MAKE) -C $(CARMEN_HOME)/sharedlib/darknet2


include ../Makefile.rules
