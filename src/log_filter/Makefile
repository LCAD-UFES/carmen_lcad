include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = "LOG FILTER"
MODULE_COMMENT = "LOG FILTER Module"

CFLAGS += -std=c11
CXXFLAGS += -std=c++11

CFLAGS += #-DPARSE_DEBUG
IFLAGS += 
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src 
LFLAGS += -lparam_interface -lbumblebee_basic_interface -lcarmenserial -lipc -lglobal `pkg-config --libs --cflags opencv`
LFLAGS += -lgps_nmea_interface -lgps_xyz_interface -lfused_odometry_interface -llocalize_ackerman_interface -lkinect_interface -lweb_cam_interface -lstereo_interface -lbase_ackerman_interface -lxsens_interface
LFLAGS += -lrotation_geometry -ltf -lBulletCollision -lBulletDynamics \
		  -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system -lm -pthread

SOURCES = log_filter_main.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES =  
PUBLIC_LIBRARIES_SO = 
PUBLIC_BINARIES = log_filter 
TARGETS = log_filter 

ifndef NO_PYTHON
TARGETS += 
endif

log_filter: log_filter_main.o 

include $(CARMEN_HOME)/src/Makefile.rules
