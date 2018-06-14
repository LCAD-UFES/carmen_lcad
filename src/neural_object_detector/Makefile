include ../Makefile.conf

MODULE_NAME = Neural Object Detector
MODULE_COMMENT = Detect vehicles using convolutional neural networks.

LINK = g++
CXXFLAGS = -std=c++11 

IFLAGS += -DUSE_OPENCV -DNO_NVML
LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet/lib
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src

LFLAGS += -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lparam_interface -ltf -lrt -lLinearMath -lboost_thread-mt \
          -lboost_signals -lboost_system -lmoving_objects_interface -llocalize_ackerman_interface -lvelodyne_camera_calibration \
          -lcamera_boxes_to_world -ldarknet `pkg-config opencv --libs` -ldbscan `pkg-config --cflags opencv` `pkg-config --libs opencv`

SOURCES = dbscan.cpp neural_object_detector.cpp neural_object_detector_main.cpp neural_object_detector_test.cpp neural_object_detector_main_new.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = neural_object_detector

TARGETS = darknet velodyne_camera_calibration camera_boxes_to_world neural_object_detector neural_object_detector_test neural_object_detector_main_new

neural_object_detector: dbscan.o neural_object_detector_main.o neural_object_detector.o 

neural_object_detector_test: neural_object_detector_test.o

neural_object_detector_main_new: neural_object_detector_main_new.o

darknet:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/darknet
	
velodyne_camera_calibration:
	$(MAKE) -C $(CARMEN_HOME)/src/velodyne_camera_calibration
	
camera_boxes_to_world:
	$(MAKE) -C $(CARMEN_HOME)/src/camera_boxes_to_world

clean: clean_velodyne_camera_calibration clean_camera_boxes_to_world clean_darknet

clean_velodyne_camera_calibration:
	$(MAKE) clean -C $(CARMEN_HOME)/src/velodyne_camera_calibration
	
clean_camera_boxes_to_world:
	$(MAKE) clean -C $(CARMEN_HOME)/src/camera_boxes_to_world
	
clean_darknet:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/darknet
	
include ../Makefile.rules

