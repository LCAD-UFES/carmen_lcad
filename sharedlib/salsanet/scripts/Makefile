include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CFLAGS += -std=c++11
CXXFLAGS += -std=gnu++11

MODULE_NAME = SalsaNet Neural Network
MODULE_COMMENT = Semantic segmentation of LiDAR point cloud

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -fopenmp
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -fopenmp -lprob_models

IFLAGS += -I/usr/include/python2.7
LFLAGS += -lpython2.7 -lpthread `pkg-config --libs opencv` `python2.7-config --libs` 

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lvelodyne_camera_calibration
LFLAGS += -lglobal -lipc -lparam_interface -lvelodyne_interface -lbumblebee_basic_interface -lvisual_tracker_interface \
		  `pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
		  -lboost_thread-mt -lrt -lboost_signals -lboost_system -lvelodyne_camera_calibration 

SOURCES = libsalsanet.cpp velodyne_to_salsanet.cpp
 
PUBLIC_INCLUDES = libsalsanet.h
PUBLIC_LIBRARIES = libsalsanet.a
PUBLIC_BINARIES = velodyne_to_salsanet

TARGETS = libprob_models.a velodyne_camera_calibration libsalsanet.a velodyne_to_salsanet
PUBLIC_LIBRARIES_SO =

libprob_models.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/prob_models/

libsalsanet.a: libsalsanet.o

velodyne_to_salsanet: libsalsanet.o velodyne_to_salsanet.o

velodyne_camera_calibration:
	$(MAKE) -C $(CARMEN_HOME)/src/velodyne_camera_calibration

clean: clean_prob_models

clean_prob_models:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/prob_models/

# https://stackoverflow.com/questions/7369145/activating-a-virtualenv-using-a-shell-script-doesnt-seem-to-work
virtualenv:
	. create_env.sh


include $(CARMEN_HOME)/src/Makefile.rules