include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CFLAGS += -std=c++11
CXXFLAGS += -std=gnu++11

MODULE_NAME = Inplace_abn Neural Network
MODULE_COMMENT = RDDF generation with images

IFLAGS += -I/usr/include/python3.6m
LFLAGS += -lpython3.6 -lpthread `pkg-config --libs opencv` `python3.6-config --libs` 

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -lglobal -lipc -lparam_interface -lvelodyne_interface -lbumblebee_basic_interface -lvisual_tracker_interface \
		  `pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
		  -lboost_thread-mt -lrt -lboost_signals -lboost_system  -lvelodyne_camera_calibration

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system

SOURCES = libinplace_abn.cpp
 
PUBLIC_INCLUDES = libinplace_abn.h
PUBLIC_LIBRARIES = libinplace_abn.a
PUBLIC_BINARIES = 

TARGETS = libinplace_abn.a
PUBLIC_LIBRARIES_SO =

libinplace_abn.a: libinplace_abn.o

# https://stackoverflow.com/questions/7369145/activating-a-virtualenv-using-a-shell-script-doesnt-seem-to-work
virtualenv:
	. create_env.sh

include $(CARMEN_HOME)/src/Makefile.rules