include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CFLAGS += -std=c++11
CXXFLAGS += -std=gnu++11

MODULE_NAME = Efficientdet Neural Network
MODULE_COMMENT = Object detection

IFLAGS += -I/usr/include/python3.5
LFLAGS += -lpython3.5m -lpthread `pkg-config --libs opencv` `python3.5-config --libs` 

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -lglobal -lipc -lparam_interface -lvelodyne_interface -lbumblebee_basic_interface -lvisual_tracker_interface \
		  `pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
		  -lboost_thread-mt -lrt -lboost_signals -lboost_system  -lvelodyne_camera_calibration

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system

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
	. create_env.sh

include $(CARMEN_HOME)/src/Makefile.rules
