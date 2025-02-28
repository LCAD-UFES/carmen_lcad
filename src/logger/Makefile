include ../Makefile.conf

MODULE_NAME = LOGGER
MODULE_COMMENT = Modules for logging, displaying, and playing back data.

LINK = g++
CFLAGS += -Wall
CXXFLAGS += -std=c++11

ifdef FOR_XAVIER  
INTERFACE_LIBS = -lxavier_interface -lparam_interface
else

INTERFACE_LIBS = -llaser_interface -llaser_ldmrs_interface -lobstacle_avoider_interface -lsimulator_ackerman_interface -llocalize_ackerman_interface \
		 -lgps_nmea_interface -lbase_ackerman_interface -lpantilt_interface -limu_interface -lkinect_interface -lbumblebee_basic_interface \
		 -lxsens_interface -lvelodyne_interface -lxsens_mtig_interface -lweb_cam_interface -lrobot_ackerman_interface -lvisual_odometry_interface \
		 -lultrasonic_filter_interface -lford_escape_hybrid_interface -lcan_dump_interface -lpi_imu_interface -lcamera_drivers -lcarmen_util

endif

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L/usr/local/carmen_boost/lib
LFLAGS += $(INTERFACE_LIBS) -lparam_interface -lglobal -lipc -lrotation_geometry -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
			-lboost_thread-mt -lrt -lboost_signals -lboost_system `pkg-config --libs opencv` -luser_preferences

SOURCES = logger.c playback_interface.c playback.cpp writelog.cpp readlog.cpp log_timestamp_repair.c logger_interface.c print_binary_log.cpp log_fix_messages.cpp

PUBLIC_INCLUDES = logger.h logger_messages.h playback_messages.h playback_interface.h writelog.h readlog.h logger_interface.h

PUBLIC_LIBRARIES = libplayback_interface.a  libwritelog.a libreadlog.a liblogger_interface.a

PUBLIC_BINARIES = logger playback playback_control log_fix_messages

TARGETS = liblogger_interface.a  libwritelog.a libreadlog.a logger libplayback_interface.a  playback log_timestamp_repair print_binary_log log_timestamp_repair log_fix_messages

ifndef NO_GRAPHICS
SOURCES += playback_control.c
IFLAGS += `$(GTK_CONFIG) --cflags`
LFLAGS += -lglobal_graphics `$(GTK_CONFIG) --libs`
TARGETS += playback_control
endif

PUBLIC_LIBRARIES_SO = libplayback_interface.so
ifndef NO_PYTHON
TARGETS += libplayback_interface.so.1
endif

logger: liblogger_interface.a libwritelog.a logger.o

libwritelog.a: writelog.o

libreadlog.a: readlog.o

liblogger_interface.a: logger_interface.o

libplayback_interface.a: playback_interface.o

libplayback_interface.so.1: playback_interface.o

playback: playback.o libreadlog.a playback_interface.o

playback_control: playback_control.o playback_interface.o

log_timestamp_repair: log_timestamp_repair.o

log_fix_messages: playback_interface.o libreadlog.a log_fix_messages.o

print_binary_log: print_binary_log.o

include ../Makefile.rules
