include ../Makefile.conf

MODULE_NAME = PI IMU
MODULE_COMMENT = Raspberry Pi Imu Driver Module

LINK = g++
CXXFLAGS += -std=c++0x

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcmt/
LFLAGS +=

LFLAGS += -lbumblebee_basic_interface -lcamera_interface -lparam_interface -lipc -lglobal -lm `pkg-config --cflags opencv` `pkg-config --libs opencv` 

SOURCES = pi_imu_client_driver.cpp pi_imu_viewer.cpp

PUBLIC_BINARIES = pi_imu_client_driver pi_imu_viewer
PUBLIC_LIBRARIES =

TARGETS = pi_imu_client_driver pi_imu_viewer

pi_imu_client_driver: pi_imu_client_driver.o  

pi_imu_viewer: pi_imu_viewer.o

include ../Makefile.rules
