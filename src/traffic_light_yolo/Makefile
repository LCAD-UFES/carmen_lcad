include ../Makefile.conf

MODULE_NAME = Traffic Light YOLO
MODULE_COMMENT = Detect traffic lights using YOLO and prior maps.

LINK = g++
CXXFLAGS = -std=c++11

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libsickldmrs2/include -DBOOST_SIGNALS_NO_DEPRECATION_WARNING

LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet2/lib

LFLAGS += -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lparam_interface
LFLAGS += -llocalize_ackerman_interface -ltraffic_light_interface -ltf
LFLAGS += -lboost_system -lboost_signals -lboost_program_options
LFLAGS += -lvelodyne_camera_calibration -lmoving_objects_interface
LFLAGS += `pkg-config --libs opencv`
LFLAGS += -ldarknet2

SOURCES = traffic_light_yolo.cpp

TARGETS = traffic_light_yolo

PUBLIC_BINARIES = traffic_light_yolo

traffic_light_yolo: traffic_light_yolo.o

darknet2:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/darknet2


include ../Makefile.rules
