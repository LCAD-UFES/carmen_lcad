include ../Makefile.conf

MODULE_NAME = "GPS XYZ"
MODULE_COMMENT = "GPS Module XYZ"

CFLAGS += #-DPARSE_DEBUG
IFLAGS += 
LFLAGS += -L../../sharedlib/libtf/src
LFLAGS += -lparam_interface -lcarmenserial -lipc -lglobal -lxsens_mtig_interface -lvelodyne_interface -lgps_nmea_interface \
		  -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt \
		  -lboost_signals -lboost_system -lstdc++ -pthread

SOURCES = gps_xyz_interface.c gps_xyz-test.cpp gps_xyz.cpp

PUBLIC_INCLUDES = gps_xyz_messages.h gps_xyz_interface.h

PUBLIC_LIBRARIES = libgps_xyz_interface.a 

PUBLIC_BINARIES = gps_xyz gps_xyz-test 

TARGETS = gps_xyz gps_xyz-test libgps_xyz_interface.a 

PUBLIC_LIBRARIES_SO = libgps_xyz_interface.so
ifndef NO_PYTHON
TARGETS += libgps_xyz_interface.so.1
endif

libgps_xyz_interface.a: 	gps_xyz_interface.o

libgps_xyz_interface.so.1: 	gps_xyz_interface.o

gps_xyz:			gps_xyz.o gps_xyz_interface.o ../gps/libgps_nmea_interface.a

gps_xyz-test:			gps_xyz-test.o gps_xyz_interface.o

# rules

include ../Makefile.rules
