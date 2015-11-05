include ../Makefile.conf

LINK=g++

LFLAGS +=  -lreadlog -lwritelog -lmap_interface -lglobal -lipc

CFLAGS += -O3
CXXFLAGS += -O3

MODULE_NAME = CARMENPP
MODULE_COMMENT = CPP wrapper tools for CARMEN

SOURCES =  *.cpp 

PUBLIC_INCLUDES  = cpp_abstractmap.h  cpp_carmenmap.h  cpp_genericpoint.h  cpp_laser.h  \
                   cpp_movement.h  cpp_simulator.h  cpp_base.h  cpp_genericmap.h  \
                   cpp_global.h  cpp_logfile.h  cpp_point.h  cpp_carmen.h \
                   cpp_abstractmessage.h  cpp_global_macros.h  cpp_mapdefinitions.h  \
                   cpp_robot.h cpp_abstractmap.hxx cpp_genericmap.hxx cpp_mapconfig.h \
                   cpp_imu.h cpp_unknownmessage.h

PUBLIC_LIBRARIES = libcarmen_carmenpp.a

PUBLIC_BINARIES  = 

TARGETS = libcarmen_carmenpp.a 

libcarmen_carmenpp.a:  cpp_laser.o cpp_base.o cpp_simulator.o cpp_robot.o cpp_carmenmap.o cpp_logfile.o cpp_mapconfig.o cpp_unknownmessage.o cpp_imu.o

include ../Makefile.rules
