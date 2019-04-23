include ../Makefile.conf

MODULE_NAME = Neural Object Detector
MODULE_COMMENT = Detect vehicles using convolutional neural networks.

LINK = g++ -O3
CXXFLAGS = -std=c++11 -O3 

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -DUSE_OPENCV -DNO_NVML -DBOOST_SIGNALS_NO_DEPRECATION_WARNING
#LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet/lib
LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet2/lib

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libsickldmrs2/include
# Required default libraries to comunicate with Carmen Core.
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libsickldmrs2 

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L$(CARMEN_HOME)/sharedlib/libtf/src -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system

LFLAGS += -lplayback_interface -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lbehavior_selector_interface -lrddf_interface -lparam_interface -ltf -lrt -lLinearMath -lboost_thread-mt \
          -lboost_signals -lboost_system -lmoving_objects_interface -llocalize_ackerman_interface -lvelodyne_camera_calibration \
          -lcamera_boxes_to_world -ldarknet2 `pkg-config opencv --libs` -ldbscan `pkg-config --cflags opencv` `pkg-config --libs opencv` \
		  -llaser_ldmrs_interface -llaser_ldmrs_utils

#SOURCES = dbscan.cpp neural_object_detector2.cpp neural_object_detector_main2_yolov3.cpp neural_object_detector_test2.cpp neural_object_detector_main_new2.cpp SampleFilter_neural_object_detector2.cpp
SOURCES = dbscan.cpp neural_object_detector2.cpp neural_object_detector_main2_yolov3.cpp neural_object_detector_main_new2.cpp SampleFilter_neural_object_detector2.cpp

PUBLIC_INCLUDES = behavior_selector_messages.h
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = neural_object_detector2

#TARGETS = darknet2 libclustering velodyne_camera_calibration camera_boxes_to_world neural_object_detector2 neural_object_detector_test2 neural_object_detector_main_new2
TARGETS = darknet2 libclustering velodyne_camera_calibration camera_boxes_to_world neural_object_detector2 neural_object_detector_main_new2

neural_object_detector2: dbscan.o SampleFilter_neural_object_detector2.o neural_object_detector_main2_yolov3.o neural_object_detector2.o 

#neural_object_detector_test2: neural_object_detector_test2.o

neural_object_detector_main_new2: neural_object_detector_main_new2.o

#darknet:
	#$(MAKE) -C $(CARMEN_HOME)/sharedlib/darknet
darknet2:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/darknet2

libclustering:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libclustering
	
velodyne_camera_calibration:
	$(MAKE) -C $(CARMEN_HOME)/src/velodyne_camera_calibration
	
camera_boxes_to_world:
	$(MAKE) -C $(CARMEN_HOME)/src/camera_boxes_to_world

clean: clean_velodyne_camera_calibration clean_camera_boxes_to_world clean_libclustering

clean_velodyne_camera_calibration:
	$(MAKE) clean -C $(CARMEN_HOME)/src/velodyne_camera_calibration
	
clean_camera_boxes_to_world:
	$(MAKE) clean -C $(CARMEN_HOME)/src/camera_boxes_to_world
	
clean_darknet:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/darknet2

clean_libclustering:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libclustering
	
include ../Makefile.rules

