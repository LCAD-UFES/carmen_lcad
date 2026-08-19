include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CFLAGS += -std=c++14
CXXFLAGS += -std=gnu++11

MODULE_NAME = EfficientDet Neural Network
MODULE_COMMENT = Object detection

IFLAGS += -I/usr/include/python3.5m -I/usr/include/python3.6 -DBOOST_SIGNALS_NO_DEPRECATION_WARNING 
UBUNTU_VERSION ="$(shell cat /etc/issue | grep Ubuntu | cut -c8-12)"
#$(warning now we reached ifeq UBUNTU_VERSION=$(UBUNTU_VERSION))
ifeq ($(UBUNTU_VERSION),"16.04")
	LFLAGS += `pkg-config --libs python3-embed 2>/dev/null || python3-config --embed --libs 2>/dev/null || python3-config --libs`
else
	LFLAGS += `pkg-config --libs python3-embed 2>/dev/null || python3-config --embed --libs 2>/dev/null || python3-config --libs`
endif

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/include/bullet/ -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -lglobal -lipc -lparam_interface -lvelodyne_interface -lbumblebee_basic_interface -lvisual_tracker_interface \
		  `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
		  -lboost_thread -lrt  -lvelodyne_camera_calibration

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread -lrt

SOURCES = libefficientdet.cpp camera_to_efficientdet.cpp
 
PUBLIC_INCLUDES = libefficientdet.h
PUBLIC_LIBRARIES = libefficientdet.a
PUBLIC_BINARIES = camera_to_efficientdet

TARGETS = libefficientdet.a camera_to_efficientdet
PUBLIC_LIBRARIES_SO =

libefficientdet.a: libefficientdet.o

camera_to_efficientdet: libefficientdet.o camera_to_efficientdet.o

# https://stackoverflow.com/questions/7369145/activating-a-virtualenv-using-a-shell-script-doesnt-seem-to-work
virtualenv:
	. install_requirements.sh

include $(CARMEN_HOME)/src/Makefile.rules
