include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CFLAGS += -std=c++11
CXXFLAGS += -std=gnu++11

MODULE_NAME = SqueezeSeg V2 Neural Network
MODULE_COMMENT = Semantic segmentation of LiDAR point cloud

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -fopenmp
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -fopenmp -lprob_models

IFLAGS += -I/usr/include/python2.7 -DBOOST_SIGNALS_NO_DEPRECATION_WARNING 
LFLAGS += -lpython2.7 -lpthread `pkg-config --libs opencv` `python2.7-config --libs` 

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/include/bullet/ -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lvelodyne_camera_calibration
LFLAGS += -lglobal -lipc -lparam_interface -lvelodyne_interface -lbumblebee_basic_interface -lvisual_tracker_interface \
		  `pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
		  -lboost_thread-mt -lrt -lboost_signals -lboost_system -lvelodyne_camera_calibration -lvelodyne_interface -lparam_interface

SOURCES = libsqueeze_seg.cpp velodyne_to_SqueezeSeg.cpp
 
PUBLIC_INCLUDES = libsqueeze_seg.h
PUBLIC_LIBRARIES = libsqueeze_seg.a
PUBLIC_BINARIES = velodyne_to_SqueezeSeg

TARGETS = libprob_models.a libsqueeze_seg.a libvelodyne_interface.a velodyne_camera_calibration velodyne_to_SqueezeSeg 
PUBLIC_LIBRARIES_SO =

libprob_models.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/prob_models/

libsqueeze_seg.a: libsqueeze_seg.o

velodyne_to_SqueezeSeg: libsqueeze_seg.o velodyne_to_SqueezeSeg.o

libvelodyne_interface.a:
	$(MAKE) -C $(CARMEN_HOME)/src/velodyne

velodyne_camera_calibration:
	$(MAKE) -C $(CARMEN_HOME)/src/velodyne_camera_calibration

clean: clean_prob_models

clean_prob_models:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/prob_models/

# https://stackoverflow.com/questions/7369145/activating-a-virtualenv-using-a-shell-script-doesnt-seem-to-work
virtualenv:
	. create_env.sh


include $(CARMEN_HOME)/src/Makefile.rules