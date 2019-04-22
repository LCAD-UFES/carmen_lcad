include ../Makefile.conf

MODULE_NAME = PI IMU
MODULE_COMMENT = Raspberry Pi Imu Driver Module

LINK = g++
CXXFLAGS += -std=c++0x

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcmt/ -I/usr/include/eigen3/ `pkg-config opencv --cflags`
LFLAGS += `pkg-config opencv --libs`

LFLAGS +=  -lparam_interface -lipc -lglobal -lm  -lglut -lGL -lGLU -lxsens_interface

SOURCES = pi_imu_client_driver.cpp pi_imu_interface.cpp pi_imu_server_logger.cpp#pi_imu_viewer.cpp


PUBLIC_BINARIES = pi_imu_client_driver#pi_imu_viewer

PUBLIC_INCLUDES = pi_imu_messages.h pi_imu_interface.h
PUBLIC_LIBRARIES = libpi_imu_interface.a
PUBLIC_BINARIES = pi_imu_client_driver pi_imu_server_logger #pi_imu_viewer

TARGETS = pi_imu_client_driver libpi_imu_interface.a pi_imu_server_logger#pi_imu_viewer

pi_imu_client_driver: pi_imu_client_driver.o  pi_imu_interface.o

pi_imu_server_logger: pi_imu_server_logger.o  pi_imu_interface.o

libpi_imu_interface.a: pi_imu_interface.o
#pi_imu_viewer: pi_imu_viewer.o

include ../Makefile.rules
