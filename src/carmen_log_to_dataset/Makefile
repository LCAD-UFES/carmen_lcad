include ../Makefile.conf

# Migração Ubuntu 26.04: o módulo linkava contra o OpenCV mas nunca pedia os -I dele.
# No Ubuntu 20.04 os headers ficavam em /usr/include/opencv2 (path padrão); o pacote do
# 26.04 põe tudo em /usr/include/opencv4, que exige -I explícito.
IFLAGS += `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv`

MODULE_NAME = Carmen Log to Dataset
MODULE_COMMENT = Carmen Log to Dataset

LINK = g++

CFLAGS += -Wall -std=c99
CXXFLAGS += -Wall -std=c++98

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lparam_interface `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv` \
-L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lrt -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
		-lboost_thread

SOURCES = carmen_log_to_dataset_main.cpp 

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 

TARGETS = carmen_log_to_dataset

carmen_log_to_dataset: carmen_log_to_dataset_main.o 

include ../Makefile.rules
