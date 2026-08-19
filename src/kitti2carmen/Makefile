include ../Makefile.conf

# Migração Ubuntu 26.04: o módulo linkava contra o OpenCV mas nunca pedia os -I dele.
# No Ubuntu 20.04 os headers ficavam em /usr/include/opencv2 (path padrão); o pacote do
# 26.04 põe tudo em /usr/include/opencv4, que exige -I explícito.
IFLAGS += `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv`

# Module name and description
MODULE_NAME = KITTI2CARMEN
MODULE_COMMENT = KITTI TO CARMEN

LINK = g++

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I$(CARMEN_HOME)/sharedlib/prob_models  -fopenmp 

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv`
# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lipc -lglobal -lvelodyne_interface -lparam_interface -lwritelog -lkinect_interface -lbumblebee_basic_interface -lxsens_interface -lgps_nmea_interface 
LFLAGS += -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread -lrt -pthread
CFLAGS += -Wall

# Source code files (.c, .cpp) 
SOURCES = kitti2carmen_main.cpp read_kitti.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = 

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = 

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = kitti2carmen_main

TARGETS = kitti2carmen_main

kitti2carmen_main: kitti2carmen_main.o read_kitti.o

include ../Makefile.rules

