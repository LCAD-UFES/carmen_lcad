include ../Makefile.conf

MODULE_NAME = PI IMU
MODULE_COMMENT = Raspberry Pi Imu Driver Module

LINK = g++
CXXFLAGS += -std=c++0x

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcmt/ -I/usr/include/eigen3/ `pkg-config opencv --cflags`
LFLAGS += `pkg-config opencv --libs`

LFLAGS +=  -lparam_interface -lipc -lglobal -lm  -lglut -lGL -lGLU 

SOURCES = pi_imu_client_driver.cpp #pi_imu_viewer.cpp

PUBLIC_BINARIES = pi_imu_client_driver #pi_imu_viewer
PUBLIC_LIBRARIES =

TARGETS = pi_imu_client_driver #pi_imu_viewer

pi_imu_client_driver: pi_imu_client_driver.o  

#pi_imu_viewer: pi_imu_viewer.o

include ../Makefile.rules
