include ../Makefile.conf

MODULE_NAME = "GPS XYZ"
MODULE_COMMENT = "GPS Module XYZ"

CFLAGS += #-DPARSE_DEBUG
IFLAGS += 

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I/usr/include/bullet/ 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L/usr/lib/x86_64-linux-gnu

LFLAGS += -L../../sharedlib/libtf/src
LFLAGS += -lparam_interface -lcarmenserial -lipc -lglobal -lxsens_mtig_interface -lvelodyne_interface -lgps_nmea_interface -lbase_ackerman_interface \
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
