include ../Makefile.conf

CFLAGS += `pkg-config --cflags opencv` -g -Wall
IFLAGS += 
LFLAGS += -lstdc++ -lglobal -lipc -lparam_interface `pkg-config --libs opencv` -lvelodyne_interface \
	-lbumblebee_basic_interface -lvisual_tracker_interface -ltf 

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system

MODULE_NAME = VELODYNE CAMERA CALIBRATION
MODULE_COMMENT = Velodyne Camera Calibration

SOURCES = velodyne_camera_calibration_main.cpp velodyne_camera_publish_range.cpp \
			velodyne_camera_calibration.cpp

PUBLIC_INCLUDES = velodyne_camera_calibration.h
PUBLIC_LIBRARIES = libvelodyne_camera_calibration.a
PUBLIC_BINARIES = 
TARGETS = velodyne_camera_calibration_main velodyne_camera_publish_range libvelodyne_camera_calibration.a

velodyne_camera_calibration_main: velodyne_camera_calibration_main.o libvelodyne_camera_calibration.a

velodyne_camera_publish_range: velodyne_camera_publish_range.o libvelodyne_camera_calibration.a

libvelodyne_camera_calibration.a: velodyne_camera_calibration.o

include ../Makefile.rules
