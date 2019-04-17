include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = SEG MAP UTIL
MODULE_COMMENT = Utilities for segmap module 

CXXFLAGS += -Wno-sign-compare -std=c++11
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11

#IFLAGS += -std=c++11
#IFLAGS += -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ -I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`
#LFLAGS += -L /usr/local/lib -lpcl_common -lpcl_io -lpcl_visualization
#LFLAGS += -L /usr/lib/x86_64-linux-gnu/ -lboost_system
#LFLAGS += -L /usr/lib -lvtkIO -lvtkFiltering -lvtkCommon -lvtkRendering -lvtkGraphics `pkg-config --libs opencv`

LFLAGS += -lboost_program_options -lsegmap_utils

SOURCES = command_line.cpp  carmen_param_file.cpp util_io.cpp util_math.cpp util_strings.cpp util_time.cpp

PUBLIC_INCLUDES = command_line.h carmen_param_file.h util_io.h util_math.h util_strings.h util_time.h
PUBLIC_LIBRARIES = libutil.a
TARGETS = libutil.a

libutil.a: util_io.o util_math.o util_strings.o util_time.o command_line.o carmen_param_file.o 

include $(CARMEN_HOME)/src/Makefile.rules
