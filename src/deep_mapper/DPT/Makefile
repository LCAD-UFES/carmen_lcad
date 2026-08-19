include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CFLAGS += -std=c++14
CXXFLAGS += -std=gnu++11

MODULE_NAME = DPT Neural Network
MODULE_COMMENT = Depth Estimation with Intel DPT

IFLAGS += -I/usr/include/python3.8
LFLAGS += `pkg-config --libs python3-embed 2>/dev/null || python3-config --embed --libs 2>/dev/null || python3-config --libs` -L /usr/lib/x86_64-linux-gnu/ -lpython3.8

UBUNTU_VERSION ="$(shell cat /etc/issue | grep Ubuntu | cut -c8-12)"

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -lglobal -lipc -lparam_interface -lvelodyne_interface -lbumblebee_basic_interface -lvisual_tracker_interface \
		  `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
		  -lboost_thread -lrt  -lvelodyne_camera_calibration

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread -lrt

SOURCES = libdpt.cpp
 
PUBLIC_INCLUDES = libdpt.h
PUBLIC_LIBRARIES = libdpt.a
PUBLIC_BINARIES = 

TARGETS = libdpt.a
PUBLIC_LIBRARIES_SO =

libdpt.a: libdpt.o

MYDIR = weights

create_weights:
	[ -d $(MYDIR) ] || mkdir -p $(MYDIR)

download: create_weights download_kitti 
			mv dpt_hybrid_kitti-cb926ef4.pt weights/

download_kitti:
	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1-oJpORoJEdxj4LTV-Pc17iB-smp-khcX/view?usp=sharing dpt_hybrid_kitti-cb926ef4.pt

virtualenv:
	. create_env.sh

include $(CARMEN_HOME)/src/Makefile.rules
