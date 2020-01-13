include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CFLAGS += -std=c++11
CXXFLAGS += -std=gnu++11

MODULE_NAME = SqueezeSeg V2 Neural Network
MODULE_COMMENT = Semantic segmentation of LiDAR point cloud

IFLAGS += -I/usr/include/python2.7
LFLAGS += -lpython2.7 -lpthread `pkg-config --libs opencv` `python2.7-config --libs` 

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -lglobal -lipc -lparam_interface -lvelodyne_interface -lbumblebee_basic_interface -lvisual_tracker_interface \
		  `pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
		  -lboost_thread-mt -lrt -lboost_signals -lboost_system  -lvelodyne_camera_calibration

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system

SOURCES = libsqueeze_seg.cpp velodyne_to_SqueezeSeg.cpp
 
PUBLIC_INCLUDES = libsqueeze_seg.h
PUBLIC_LIBRARIES = libsqueeze_seg.a
PUBLIC_BINARIES = velodyne_to_SqueezeSeg

TARGETS = libsqueeze_seg.a velodyne_camera_calibration #velodyne_to_SqueezeSeg
PUBLIC_LIBRARIES_SO =

libsqueeze_seg.a: libsqueeze_seg.o

velodyne_to_SqueezeSeg: libsqueeze_seg.o velodyne_to_SqueezeSeg.o

velodyne_camera_calibration:
	$(MAKE) -C $(CARMEN_HOME)/src/velodyne_camera_calibration

# https://stackoverflow.com/questions/7369145/activating-a-virtualenv-using-a-shell-script-doesnt-seem-to-work
virtualenv:
	. create_env.sh


include $(CARMEN_HOME)/src/Makefile.rules