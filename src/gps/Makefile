include ../Makefile.conf

MODULE_NAME = "GPS"
MODULE_COMMENT = "NMEA GPS Module"

LINK=g++

#CFLAGS += -DGPGGA_PARSE_DEBUG
IFLAGS += 
LFLAGS += -lparam_interface -lcarmenserial -lipc -lglobal -lstdc++

SOURCES = gps-io.c gps-ipc.c gps-nmea.c gps-main.c \
	  gps_nmea_interface.c gps-test.cpp gps-trimble-main.cpp

PUBLIC_INCLUDES = gps_nmea_messages.h gps_nmea_interface.h

PUBLIC_LIBRARIES = libgps_nmea_interface.a 

PUBLIC_BINARIES = gps_nmea gps_test gps_nmea_trimble

TARGETS = gps_nmea gps_nmea_trimble libgps_nmea_interface.a gps_test 

PUBLIC_LIBRARIES_SO = libgps_nmea_interface.so
ifndef NO_PYTHON
TARGETS += libgps_nmea_interface.so.1
endif

gps_nmea:			gps-io.o gps-ipc.o gps-nmea.o gps-main.o

gps_nmea_trimble:	gps-ipc.o gps-nmea.o gps_nmea_interface.o gps-trimble-main.o

gps_test:			gps-test.o gps_nmea_interface.o

libgps_nmea_interface.a:	gps_nmea_interface.o

libgps_nmea_interface.so.1:	gps_nmea_interface.o

# rules

include ../Makefile.rules
