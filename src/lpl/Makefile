include ../Makefile.conf

MODULE_NAME = Neural Object Detector
MODULE_COMMENT = Detect vehicles using convolutional neural networks.

LINK = g++
CXXFLAGS = -std=c++11

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/include/bullet/ -I/usr/local/include/bullet/ -I/usr/local/carmen_boost/include -I/usr/include/python3.8 -DBOOST_SIGNALS_NO_DEPRECATION_WARNING 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L/usr/local/carmen_boost/lib

#IFLAGS += -I$(CARMEN_HOME)/sharedlib/darknet2
#LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet2/lib -L$(CARMEN_HOME)/sharedlib/libtf/src

LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet3/lib -L$(CARMEN_HOME)/sharedlib/libtf/src

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libsickldmrs2/include
# Required default libraries to comunicate with Carmen Core.
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libsickldmrs2 

#LFLAGS += -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lparam_interface -llocalize_ackerman_interface \
		  -lmoving_objects_interface -ldarknet2 `pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics \
		  -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system -lvelodyne_camera_calibration \
		  -ltraffic_light_interface -llaser_ldmrs_interface -llaser_ldmrs_utils -lrddf_interface `python3.8-config --libs` \
		  -lcamera_drivers -lpython3.8

LFLAGS += -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lparam_interface -llocalize_ackerman_interface \
		  -lmoving_objects_interface -ldarknet `pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics \
		  -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system -lvelodyne_camera_calibration \
		  -ltraffic_light_interface -llaser_ldmrs_interface -llaser_ldmrs_utils -lrddf_interface `python3.8-config --libs` \
		  -lcamera_drivers -lpython3.8

SOURCES = neural_object_detector_tracker.cpp

PUBLIC_BINARIES = neural_object_detector_tracker

TARGETS = darknet3 neural_object_detector_tracker

neural_object_detector_tracker: neural_object_detector_tracker.o

darknet3:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/darknet3

download:
#	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1rE0lY4d4wyMp4mvcx23y4qGK97bvzTnC/view?usp=sharing $(CARMEN_HOME)/sharedlib/darknet3/yolov3.weights
	mkdir pedestrian_tracker/data; $(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1MgaW3ndzOiKWJI8VVkAgTLL4tHzDpxc8/view?usp=sharing $(CARMEN_HOME)/src/neural_object_detector3/pedestrian_tracker/data/googlenet_part8_all_xavier_ckpt_56.h5
	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1a84AIWEsFufsXplKRS26z7FsxNt31KUb/view?usp=sharing $(CARMEN_HOME)/src/neural_object_detector3/pedestrian_tracker/data/squeezenet_small40_coco_mot16_ckpt_10.h5
	cd $(CARMEN_HOME)/sharedlib/darknet3; wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights

include ../Makefile.rules
