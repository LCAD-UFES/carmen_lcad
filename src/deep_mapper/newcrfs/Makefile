include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CFLAGS += -std=c++14
CXXFLAGS += -std=gnu++11 

MODULE_NAME = NeWCRFs Neural Network
MODULE_COMMENT = Depth Estimation with NeWCRFs

IFLAGS += -I/usr/include/python3.8
LFLAGS += `pkg-config --libs python3-embed 2>/dev/null || python3-config --embed --libs 2>/dev/null || python3-config --libs`

UBUNTU_VERSION ="$(shell cat /etc/issue | grep Ubuntu | cut -c8-12)"

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -lglobal -lipc -lparam_interface -lvelodyne_interface -lbumblebee_basic_interface -lvisual_tracker_interface \
		  `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
		  -lboost_thread -lrt  -lvelodyne_camera_calibration

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread -lrt

SOURCES = libnewcrfs.cpp

PUBLIC_INCLUDES = libnewcrfs.h
PUBLIC_LIBRARIES = libnewcrfs.a
PUBLIC_BINARIES = 

TARGETS = libnewcrfs.a
PUBLIC_LIBRARIES_SO =

libnewcrfs.a: libnewcrfs.o

MYDIR = ckpt

create_ckpt:
	[ -d $(MYDIR) ] || mkdir -p $(MYDIR)

download: create_ckpt download_kitti 
			mv model_kittieigen.ckpt ckpt/

download_kitti:
	$(CARMEN_HOME)/bin/gdown.pl https://virutalbuy-public.oss-cn-hangzhou.aliyuncs.com/share/newcrfs/models/model_kittieigen.ckpt model_kittieigen.ckpt


# https://stackoverflow.com/questions/7369145/activating-a-virtualenv-using-a-shell-script-doesnt-seem-to-work
virtualenv:
	. create_env.sh

include $(CARMEN_HOME)/src/Makefile.rules
