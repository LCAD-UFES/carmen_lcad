include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CFLAGS += -std=c++11
CXXFLAGS += -std=gnu++11

MODULE_NAME = Adabins Neural Network
MODULE_COMMENT = Depth Estimation with Adabins

IFLAGS += -I/usr/include/python3.7 -I/usr/include/python3.6m
LFLAGS += `python3.7-config --libs`

UBUNTU_VERSION ="$(shell cat /etc/issue | grep Ubuntu | cut -c8-12)"

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -lglobal -lipc -lparam_interface -lvelodyne_interface -lbumblebee_basic_interface -lvisual_tracker_interface \
		  `pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
		  -lboost_thread-mt -lrt -lboost_signals -lboost_system  -lvelodyne_camera_calibration

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system

SOURCES = libadabins.cpp
 
PUBLIC_INCLUDES = libadabins.h
PUBLIC_LIBRARIES = libadabins.a
PUBLIC_BINARIES = 

TARGETS = libadabins.a
PUBLIC_LIBRARIES_SO =

libadabins.a: libadabins.o

MYDIR = pretrained

create_pretrained:
	[ -d $(MYDIR) ] || mkdir -p $(MYDIR)

download: create_pretrained download_kitti download_nyu

download_kitti:
	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1HMgff-FV6qw1L0ywQZJ7ECa9VPq1bIoj/edit AdaBins_kitti.pt
	
download_nyu:
	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1lvyZZbC9NLcS8a__YPcUP7rDiIpbRpoF/edit AdaBins_nyu.pt

# https://stackoverflow.com/questions/7369145/activating-a-virtualenv-using-a-shell-script-doesnt-seem-to-work
virtualenv:
	. create_env.sh

include $(CARMEN_HOME)/src/Makefile.rules
