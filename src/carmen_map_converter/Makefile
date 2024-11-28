include ../Makefile.conf

CFLAGS += -std=c11 -fopenmp # -fsanitize=address -fno-omit-frame-pointer -fsanitize=leak -fsanitize=undefined
CXXFLAGS += -std=c++11 -fopenmp # -fsanitize=address -fno-omit-frame-pointer -fsanitize=leak -fsanitize=undefined
LINK = g++ # -fsanitize=address -fno-omit-frame-pointer -fsanitize=leak -fsanitize=undefined
CXXFLAGS += -Wno-parentheses -Wno-deprecated-copy -Wno-implicit-fallthrough

IFLAGS += -I$(CARMEN_HOME)/sharedlib/gmapping -I$(CARMEN_HOME)/sharedlib/gmapping/sensor
LFLAGS += -L$(CARMEN_HOME)/sharedlib/gmapping/lib

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I/usr/include/bullet/ -I/usr/local/carmen_boost/include
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L/usr/local/carmen_boost/lib

LFLAGS += -lglobal -lparam_interface -lipc \
	-llocalize_ackerman_interface -lmapper_interface -lmapper -lprob_models \
	-lmap_interface -lmap_util -lrobot_ackerman_interface \
	-lobstacle_avoider_interface -lsimulator_ackerman_interface \
	-ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system \
	-lrotation_geometry -lvelodyne_interface -lfused_odometry_interface -llocalize_ackerman_core -lmap_io -llocalize_ackerman_motion \
	-lbehavior_selector_interface -lcarmodel -lgrid_mapping -lmap_io -ltask_manager_interface -lstereo_velodyne -lbase_ackerman_interface \
	-lxsens_interface -lstereo_velodyne_interface -lstereo_interface -lplayback_interface -lmap_server_interface -lgps_xyz_interface \
	-lmoving_objects_interface -ldbscan -lgsl -lgslcblas `pkg-config --libs opencv` -llocalize_ackerman_trailers_theta

#Lib_Neural_Mapper_py
IFLAGS += -I/usr/include/python3.5m -I/usr/include/python3.6
UBUNTU_VERSION ="$(shell cat /etc/issue | grep Ubuntu | cut -c8-12)"
#$(warning now we reached ifeq UBUNTU_VERSION=$(UBUNTU_VERSION))
ifeq ($(UBUNTU_VERSION),"16.04")
	LFLAGS += `python3.5-config --libs`
else ifeq ($(UBUNTU_VERSION),"18.04")
	LFLAGS += `python3.6-config --libs`
else
	LFLAGS += `python3.8-config --embed --libs`
	IFLAGS += -I/usr/include/python3.8
endif

SOURCES = carmen_map_converter_main.cpp 

PUBLIC_BINARIES = carmen_map_converter_main

TARGETS = carmen_map_converter_main libprob_models.a


ifndef NO_PYTHON
TARGETS += 
endif

ifdef CARMEN_DEBUG
CFLAGS += -DDEBUG
endif
 
carmen_map_converter_main: carmen_map_converter_main.o

# rules

libprob_models.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/prob_models/

clean: clean_prob_models

clean_prob_models:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/prob_models/
	
include ../Makefile.rules

