
include ../Makefile.conf

# Module name and description
MODULE_NAME = DQN
MODULE_COMMENT = Deep Q-Network

LINK = g++

# Required default libraries to comunicate with Carmen Core
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -I../car_panel_gui/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -L../car_panel_gui/

# Caffe 
IFLAGS += -I$(CAFFE_HOME)/include -I$(CAFFE_HOME)/build/include/
LFLAGS += -L$(CAFFE_HOME)/build/lib
# Caffe (CPU) - comment the lines below and uncomment the lines after if you will use CPU
#CFLAGS += -DCPU_ONLY 
#CXXFLAGS += -DCPU_ONLY  
# Caffe (GPU) - comment the lines below and uncomment the lines above if you will use CPU 
IFLAGS += -I/usr/local/cuda/include
LFLAGS += -L/usr/local/cuda/lib64

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lipc -lglobal `pkg-config --libs opencv` -lgrid_mapping_interface -lrddf_interface \
	-llocalize_ackerman_interface -lrobot_ackerman_interface -lrddf_index -lprob_models -lmap_server_interface \
	-lcollision_detection -lmap_interface -lrotation_geometry -lgrid_mapping -lmap_io -lboost_system -lglog -lcaffe
CFLAGS += -Wall -Wno-write-strings -fopenmp #-pg
CXXFLAGS += -Wall -Wno-write-strings -fopenmp #-pg

# Source code files (.c, .cpp) 
SOURCES = dqn_emulator.cpp dqn_caffe.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = 

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = 

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = dqn_emulator 

TARGETS = dqn_emulator

dqn_emulator: dqn_emulator.o dqn_caffe.o

# rules

include ../Makefile.rules

