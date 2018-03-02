include ../../Makefile.conf

MODULE_NAME = Neural Mapper
MODULE_COMMENT = Creates a map based on velodyne sensor with a neural network

LINK = g++

CFLAGS += #-Wall
CXXFLAGS = -Wno-sign-compare -Wno-write-strings -DUSE_OPENCV 

IFLAGS += `pkg-config opencv --cflags` -std=c++11 -DNO_NVML
LFLAGS += `pkg-config opencv --libs` -std=c++11

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -fopenmp
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -fopenmp

IFLAGS += -I $(CAFFE_HOME)/include -I $(CAFFE_HOME)/build/src/ -I $(CAFFE_HOME)/3rdparty -I $(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -lglobal -lipc -lvelodyne_interface -lparam_interface -lmapper_interface -lrotation_geometry \
		-L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lrt -lbase_ackerman_interface \
		-lLinearMath \
		-lboost_thread-mt -lrt -lboost_signals -lboost_system \
		-llocalize_ackerman_interface \
		-lgrid_mapping -lmapper -lmap_io -lmap_util -lprob_models

LFLAGS += -L $(CAFFE_HOME)/build/lib \
		-lglog -lprotobuf -L/usr/local/lib

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lmap_interface
SOURCES = neural_mapper_dataset_generator.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES =

TARGETS = neural_mapper_dataset_generator libprob_models.a

#rules
libprob_models.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/prob_models/

neural_mapper_dataset_generator: neural_mapper_dataset_generator.o
clean: clean_prob_models

clean_prob_models:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/prob_models/

include ../../Makefile.rules

