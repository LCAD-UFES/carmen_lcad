include ../Makefile.conf

MODULE_NAME = Carmen Log to Dataset
MODULE_COMMENT = Carmen Log to Dataset

LINK = g++

CFLAGS += -Wall -std=c99
CXXFLAGS += -Wall -std=c++98

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lparam_interface `pkg-config --libs opencv` \
-L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lrt -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
		-lboost_thread-mt -lboost_signals -lboost_system

SOURCES = carmen_log_to_dataset_main.cpp 

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 

TARGETS = carmen_log_to_dataset

carmen_log_to_dataset: carmen_log_to_dataset_main.o 

include ../Makefile.rules
