include ../Makefile.conf

ifndef CARMEN_DEBUG
CXXFLAGS += -Wall -O3 -fopenmp
else
CXXFLAGS += -Wall -O3
endif

IFLAGS += -I/usr/local/include -I./
LFLAGS += -L/usr/local/lib
LFLAGS += -lglobal -lparam_interface -lipc -lgsl -lGL -lGLU -lglut \
			-lstereo_interface -lv_disparity \
			-lm -lgslcblas -fopenmp \
			`pkg-config --libs opencv`

MODULE_NAME = ROAD_FINDING
MODULE_COMMENT = Road Finding Library and Viewer 

LINK = g++

SOURCES = ml_road_finding_test.cpp ml_road_finding.cpp image_utils.cpp gsl_utils.cpp time_profile.cpp
PUBLIC_INCLUDES = ml_road_finding.h time_profile.h ml_road_finding_basic.h gsl_utils.h

ifndef NO_CUDA
SOURCES += ml_road_finding_GPU.cu
endif

PUBLIC_LIBRARIES = libroad_finding.a libtime_profile.a
PUBLIC_BINARIES = ml_road_finding_test
MAN_PAGES =

#TARGETS = ml_road_finding_test libroad_finding.a libtime_profile.a
TARGETS = libroad_finding.a libtime_profile.a

PUBLIC_LIBRARIES_SO =

ifndef NO_PYTHON
TARGETS += 
endif

ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += -lparam_interface -lipc -lglobal -lglobal_graphics \
	`$(GTK_CONFIG) --libs` 
SOURCES += road_finding_view.cpp
PUBLIC_BINARIES += road_finding_view
TARGETS += road_finding_view
endif

ml_road_finding_test: ml_road_finding_test.o libtime_profile.a libroad_finding.a

road_finding_view: road_finding_view.o libtime_profile.a libroad_finding.a

ifndef NO_CUDA
libroad_finding.a: ml_road_finding.o image_utils.o gsl_utils.o ml_road_finding_GPU.o
else
libroad_finding.a: ml_road_finding.o image_utils.o gsl_utils.o
endif

libtime_profile.a: time_profile.o

include ../Makefile.rules

