include ../Makefile.conf

MODULE_NAME = Neural Object Detector
MODULE_COMMENT = Detect vehicles using convolutional neural networks.

LINK = g++ -O3
CXXFLAGS = -std=c++11 -O3 

IFLAGS += -DUSE_OPENCV -DNO_NVML -I/usr/include/python2.7
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet

LFLAGS += -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lparam_interface -lmoving_objects_interface -llocalize_ackerman_interface \
          -lvelodyne_camera_calibration -ldarknet -ltf -lrt -lLinearMath -lboost_thread-mt -lboost_signals  -lboost_system `pkg-config opencv --libs` -ldbscan  \
          `pkg-config --cflags opencv` -lpython2.7 -lpthread

SOURCES = neural_object_detector.cpp neural_object_detector_main.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = neural_object_detector

TARGETS = darknet libclustering velodyne_camera_calibration neural_object_detector

neural_object_detector: neural_object_detector_main.o neural_object_detector.o 

darknet:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/darknet

libclustering:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libclustering
	
velodyne_camera_calibration:
	$(MAKE) -C $(CARMEN_HOME)/src/velodyne_camera_calibration

clean: clean_libclustering clean_velodyne_camera_calibration #clean_darknet

#clean_darknet:
#	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/darknet

clean_libclustering:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libclustering

clean_velodyne_camera_calibration:
	$(MAKE) clean -C $(CARMEN_HOME)/src/velodyne_camera_calibration

include ../Makefile.rules

