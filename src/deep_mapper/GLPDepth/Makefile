include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CFLAGS += -std=c++11
CXXFLAGS += -std=gnu++11 -std=gnu++14

MODULE_NAME = GLPDepth Neural Network
MODULE_COMMENT = Depth Estimation with GLPDepth

IFLAGS += -I/usr/include/python3.6m -I/usr/include/python3.7
LFLAGS += `python3.6-config --libs` `python3.7-config --libs`

#LIBTORCH
IFLAGS += -I/dados/libtorch/include -I/dados/libtorch/include/torch/csrc/api/include
LFLAGS += -L/dados/libtorch/lib -Wl,-R/dados/libtorch/lib -ltorch -ltorch_cpu -lc10 


UBUNTU_VERSION ="$(shell cat /etc/issue | grep Ubuntu | cut -c8-12)"

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -lglobal -lipc -lparam_interface -lvelodyne_interface -lbumblebee_basic_interface -lvisual_tracker_interface \
		  `pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
		  -lboost_thread-mt -lrt -lboost_signals -lboost_system  -lvelodyne_camera_calibration

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system

SOURCES = libglpdepth.cpp libpythonglpdepth.cpp
 
PUBLIC_INCLUDES = libglpdepth.h libpythonglpdepth.h 
PUBLIC_LIBRARIES = libglpdepth.a libpythonglpdepth.a
PUBLIC_BINARIES = 

TARGETS = libglpdepth.a libpythonglpdepth.a
PUBLIC_LIBRARIES_SO =

libglpdepth.a: libglpdepth.o

libpythonglpdepth.a: libpythonglpdepth.o

MYDIR = ckpt

create_ckpt:
	[ -d $(MYDIR) ] || mkdir -p $(MYDIR)

download: create_ckpt download_kitti 
			mv best_model_kitti.ckpt ckpt/

download_kitti:
	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1dipQoGbTTa6LOEPTBvG5Z66rBCl8t9W7/view?usp=sharing best_model_kitti.ckpt


# https://stackoverflow.com/questions/7369145/activating-a-virtualenv-using-a-shell-script-doesnt-seem-to-work
virtualenv:
	. create_env.sh

include $(CARMEN_HOME)/src/Makefile.rules
