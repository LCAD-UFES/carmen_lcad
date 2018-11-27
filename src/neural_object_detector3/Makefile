include ../Makefile.conf

MODULE_NAME = Neural Object Detector
MODULE_COMMENT = Detect vehicles using convolutional neural networks.

LINK = g++
CXXFLAGS = -std=c++11

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -DBOOST_SIGNALS_NO_DEPRECATION_WARNING

LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet2/lib -L$(CARMEN_HOME)/sharedlib/libtf/src

LFLAGS += -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lparam_interface -llocalize_ackerman_interface \
		  -lmoving_objects_interface -ldarknet2 `pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics \
		  -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system -lvelodyne_camera_calibration \
		  -ltraffic_light_interface

SOURCES = neural_object_detector_main.cpp neural_object_detector_simulator.cpp neural_object_detector_traffic_light.cpp

PUBLIC_BINARIES = neural_object_detector neural_object_detector_simulator neural_object_detector_traffic_light

TARGETS = darknet2 neural_object_detector neural_object_detector_simulator neural_object_detector_traffic_light

neural_object_detector: neural_object_detector_main.o

neural_object_detector_simulator: neural_object_detector_simulator.o

neural_object_detector_traffic_light: neural_object_detector_traffic_light.o

darknet2:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/darknet2


include ../Makefile.rules