include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CFLAGS += -std=c++11
CXXFLAGS += -std=gnu++11

MODULE_NAME = DPT Neural Network
MODULE_COMMENT = Depth Estimation with Intel DPT

IFLAGS += -I/usr/include/python3.7 -I/usr/include/python3.6m
LFLAGS += `python3.7-config --libs`

UBUNTU_VERSION ="$(shell cat /etc/issue | grep Ubuntu | cut -c8-12)"

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -lglobal -lipc -lparam_interface -lvelodyne_interface -lbumblebee_basic_interface -lvisual_tracker_interface \
		  `pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
		  -lboost_thread-mt -lrt -lboost_signals -lboost_system  -lvelodyne_camera_calibration

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system

SOURCES = libdpt.cpp
 
PUBLIC_INCLUDES = libdpt.h
PUBLIC_LIBRARIES = libdpt.a
PUBLIC_BINARIES = 

TARGETS = libdpt.a
PUBLIC_LIBRARIES_SO =

libdpt.a: libdpt.o

download:
	$(CARMEN_HOME)/bin/gdown.pl https://github.com/intel-isl/DPT/releases/download/1_0/dpt_hybrid-midas-501f0c75.pt weights/dpt_hybrid-midas-501f0c75.pt

# https://stackoverflow.com/questions/7369145/activating-a-virtualenv-using-a-shell-script-doesnt-seem-to-work
virtualenv:
	. create_env.sh

include $(CARMEN_HOME)/src/Makefile.rules
