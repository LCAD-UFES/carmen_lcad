include ../Makefile.conf

MODULE_NAME = Neural Object Detector
MODULE_COMMENT = Detect vehicles using convolutional neural networks.

LINK = g++ -O3
CXXFLAGS = -std=c++11 -O3 

IFLAGS += -DUSE_OPENCV -DNO_NVML
LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet/lib
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src

LFLAGS += -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lbehavior_selector_interface -lparam_interface -ltf -lrt -lLinearMath -lboost_thread-mt \
          -lboost_signals -lboost_system -lmoving_objects_interface -llocalize_ackerman_interface -lvelodyne_camera_calibration \
          -lcamera_boxes_to_world -ldarknet `pkg-config opencv --libs` -ldbscan `pkg-config --cflags opencv` `pkg-config --libs opencv`

SOURCES = dbscan.cpp neural_object_detector2.cpp neural_object_detector_main2.cpp neural_object_detector_test2.cpp neural_object_detector_main_new2.cpp

PUBLIC_INCLUDES = behavior_selector_messages.h
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = neural_object_detector2

TARGETS = darknet libclustering velodyne_camera_calibration camera_boxes_to_world neural_object_detector2 neural_object_detector_test2 neural_object_detector_main_new2

neural_object_detector2: dbscan.o neural_object_detector_main2.o neural_object_detector2.o 

neural_object_detector_test2: neural_object_detector_test2.o

neural_object_detector_main_new2: neural_object_detector_main_new2.o

darknet:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/darknet

libclustering:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libclustering
	
velodyne_camera_calibration:
	$(MAKE) -C $(CARMEN_HOME)/src/velodyne_camera_calibration
	
camera_boxes_to_world:
	$(MAKE) -C $(CARMEN_HOME)/src/camera_boxes_to_world

clean: clean_velodyne_camera_calibration clean_camera_boxes_to_world clean_darknet clean_libclustering

clean_velodyne_camera_calibration:
	$(MAKE) clean -C $(CARMEN_HOME)/src/velodyne_camera_calibration
	
clean_camera_boxes_to_world:
	$(MAKE) clean -C $(CARMEN_HOME)/src/camera_boxes_to_world
	
clean_darknet:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/darknet

clean_libclustering:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libclustering
	
include ../Makefile.rules

