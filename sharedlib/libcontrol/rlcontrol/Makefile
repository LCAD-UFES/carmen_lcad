
include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = RLCONTROL
MODULE_COMMENT = Low Level Control Library - Reinforcement Learning Controller

LINK = g++

# Required default libraries to comunicate with Carmen Core
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -I../car_panel_gui/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -L../car_panel_gui/

CXXFLAGS += -O3 -Wall -Wno-sign-compare -Wno-write-strings -Wno-unused-result -std=c++0x
CFLAGS += -O3 -Wall -Wno-sign-compare -Wno-write-strings -Wno-unused-result -std=c++0x 
IFLAGS += -I $(CAFFE_HOME)/include -I $(CAFFE_HOME)/build/src/ -I $(CAFFE_HOME)/build/include -I /usr/local/cuda/include `pkg-config --cflags opencv`
LFLAGS += -L $(CAFFE_HOME)/build/lib -lcaffe -L /usr/local/cuda/lib64 `pkg-config --libs opencv` -lglog -lboost_system -lm 

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lipc -lglobal `pkg-config --libs opencv` -lmapper_interface -lrddf_interface \
	-llocalize_ackerman_interface -lrobot_ackerman_interface -lrddf_index -lprob_models -lgrid_mapping -lmap_io -lmap_server_interface \
	-lcollision_detection -lmap_interface -lrotation_geometry -lbase_ackerman_interface -lfused_odometry_interface -lnavigator_ackerman_interface \
	-lbase_ackerman_interface 

PUBLIC_INCLUDES = rlcontrol_interface.h rlcontrol_messages.h
PUBLIC_LIBRARIES = librlcontroller_interface.a
PUBLIC_BINARIES = rlcontrol_train

SOURCE = Util.cpp ActorCriticController.cpp QLearningController.cpp rlcontrol_train.cpp rlcontrol_interface.cpp
TARGETS = rlcontrol_train librlcontroller_interface.a

rlcontrol_train: Util.o ActorCriticController.o  QLearningController.o rlcontrol_train.o rlcontrol_interface.o 
librlcontroller_interface.a: rlcontrol_interface.o

include $(CARMEN_HOME)/src/Makefile.rules

