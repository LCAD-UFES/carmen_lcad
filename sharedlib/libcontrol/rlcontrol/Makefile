
include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = RLCONTROL
MODULE_COMMENT = Low Level Control Library - Reinforcement Learning Controller

LINK = g++

# Required default libraries to comunicate with Carmen Core
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol/pid
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcarneuralmodel
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -I../car_panel_gui/

LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -L../car_panel_gui/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcarneuralmodel
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcontrol/pid

CXXFLAGS += -Wall -Wno-sign-compare -Wno-write-strings -Wno-unused-result -std=c++0x -Wno-ignored-qualifiers
CFLAGS += -Wall -Wno-sign-compare -Wno-write-strings -Wno-unused-result -std=c++0x -Wno-ignored-qualifiers
IFLAGS += -I $(CAFFE_HOME)/include -I $(CAFFE_HOME)/build/src/ -I $(CAFFE_HOME)/build/include -I /usr/local/cuda/include `pkg-config --cflags opencv`
LFLAGS += -L $(CAFFE_HOME)/build/lib -lcaffe -L /usr/local/cuda/lib64 `pkg-config --libs opencv` -lglog -lboost_system -lm 

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lipc -lglobal `pkg-config --libs opencv` -lmapper_interface -lrddf_interface \
	-llocalize_ackerman_interface -lrobot_ackerman_interface -lrddf_index -lprob_models -lgrid_mapping -lmap_io -lmap_server_interface \
	-lcollision_detection -lmap_interface -lrotation_geometry -lbase_ackerman_interface -lfused_odometry_interface -lnavigator_ackerman_interface \
	-lbase_ackerman_interface -lfann -lcarneuralmodel -lpid

PUBLIC_INCLUDES = rlcontrol_interface.h rlcontrol_messages.h
PUBLIC_LIBRARIES = librlcontroller_interface.a
PUBLIC_BINARIES = rlcontrol_train rlcontrol_train_with_ann

SOURCE = Util.cpp ActorCriticController.cpp QLearningController.cpp rlcontrol_train.cpp rlcontrol_interface.cpp rlcontrol_train_with_ann.cpp
TARGETS = libcarneuralmodel libpid rlcontrol_train librlcontroller_interface.a rlcontrol_train_with_ann

libcarneuralmodel:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libcarneuralmodel
clean: clean_libcarneuralmodel
clean_libcarneuralmodel:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libcarneuralmodel

libpid:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libcontrol/pid/
clean: clean_libpid
clean_libpid:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libcontrol/pid/

rlcontrol_train_with_ann: rlcontrol_train_with_ann.o  QLearningController.o Util.o ActorCriticController.o
rlcontrol_train: Util.o ActorCriticController.o  QLearningController.o rlcontrol_train.o rlcontrol_interface.o 
librlcontroller_interface.a: rlcontrol_interface.o

include $(CARMEN_HOME)/src/Makefile.rules

