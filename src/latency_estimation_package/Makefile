include ../Makefile.conf

MODULE_NAME = latency_estimation_package
MODULE_COMMENT = This package implements facilities for measuring latencies between sensors.

CFLAGS +=
IFLAGS += 
LFLAGS += -lglobal -lipc -lbase_interface -lparam_interface \
	  -lxsens_interface -lgps_xyz_interface -lvisual_odometry_interface -llaser_interface -lm

SOURCES = latency_estimation_module.c

PUBLIC_INCLUDES =

PUBLIC_LIBRARIES =

MAN_PAGES =

ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags`
IFLGAS += -I/usr/include/opencv  
LFLAGS += -lglobal_graphics `$(GTK_CONFIG) --libs`
LFLAGS += -lcv -lcxcore  

PUBLIC_BINARIES = latency_estimation_module
TARGETS = latency_estimation_module
endif

# rules
latency_estimation_module:	latency_estimation_module.o

include ../Makefile.rules



