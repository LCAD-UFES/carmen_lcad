include ../Makefile.conf

MODULE_NAME = Neural Object Detector
MODULE_COMMENT = Detect vehicles using convolutional neural networks.

LINK = g++
CXXFLAGS = -std=c++14

IFLAGS += -DUSE_OPENCV -DNO_NVML `pkg-config --cflags python3 2>/dev/null || python3-config --includes` -DBOOST_SIGNALS_NO_DEPRECATION_WARNING
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I/usr/include/bullet/ -DUSE_OPENCV -DNO_NVML
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libclustering
LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet/lib
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src

LFLAGS += -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lparam_interface -ltf -lrt -lLinearMath -lboost_thread \
          -lmoving_objects_interface -llocalize_ackerman_interface -lvelodyne_camera_calibration \
          -lcamera_boxes_to_world `pkg-config opencv4 --libs 2>/dev/null || pkg-config opencv --libs` -ldarknet -ldbscan `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv` `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv` `pkg-config --libs python3-embed 2>/dev/null || python3-config --embed --libs`  -lpthread -lrddf_interface 

SOURCES = dbscan.cpp neural_object_detector.cpp neural_object_detector_main.cpp neural_object_detector_test.cpp neural_object_detector_main_new.cpp traffic_light.cpp

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread -lrt

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = neural_object_detector

#TARGETS = darknet libclustering velodyne_camera_calibration camera_boxes_to_world neural_object_detector neural_object_detector_test neural_object_detector_main_new traffic_light
TARGETS = darknet libclustering camera_boxes_to_world neural_object_detector neural_object_detector_test neural_object_detector_main_new traffic_light

neural_object_detector: dbscan.o neural_object_detector_main.o neural_object_detector.o 

neural_object_detector_test: neural_object_detector_test.o

neural_object_detector_main_new: neural_object_detector_main_new.o

traffic_light: traffic_light.o

darknet:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/darknet

libclustering:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libclustering
	
#velodyne_camera_calibration:
#	$(MAKE) -C $(CARMEN_HOME)/src/velodyne_camera_calibration
	
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

