include ../Makefile.conf

MODULE_NAME = "LOG FILTER"
MODULE_COMMENT = "LOG FILTER Module"

CFLAGS += #-DPARSE_DEBUG
IFLAGS += 
LFLAGS += -lparam_interface -lbumblebee_basic_interface -lcarmenserial -lipc -lglobal `pkg-config --libs --cflags opencv`
LFLAGS +=  -lgps_nmea_interface -lgps_xyz_interface -lfused_odometry_interface -lkinect_interface -lweb_cam_interface -lstereo_interface -lbase_ackerman_interface -lxsens_interface

SOURCES = log_filter_main.c

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES =  
PUBLIC_LIBRARIES_SO = 
PUBLIC_BINARIES = log_filter 
TARGETS = log_filter 

ifndef NO_PYTHON
TARGETS += 
endif

log_filter: log_filter_main.o ../kinect/kinect_util.o

include ../Makefile.rules
