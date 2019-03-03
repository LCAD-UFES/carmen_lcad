include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = LIB_SEG_MAP
MODULE_COMMENT = Library for the segmap module

LINK = g++ 

CXXFLAGS += -pg -fopenmp
LFLAGS += -pg -fopenmp
IFLAGS += -pg 

CXXFLAGS += -Wno-deprecated -Wno-write-strings -Wno-unused-result -Wno-sign-compare -std=c++11 

# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11
IFLAGS += -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ -I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`
LFLAGS += -L /usr/local/lib -lpcl_common -lpcl_io -lpcl_visualization
LFLAGS += -L /usr/lib/x86_64-linux-gnu/ -lboost_system
LFLAGS += -L /usr/lib -lvtkIO -lvtkFiltering -lvtkCommon -lvtkRendering -lvtkGraphics -lglobal
LFLAGS += `pkg-config --libs opencv` 

SOURCES = segmap_grid_map.cpp segmap_util.cpp segmap_particle_filter.cpp segmap_pose2d.cpp \
	segmap_dataset.cpp segmap_viewer.cpp carmen_dataset_test.cpp

PUBLIC_INCLUDES = segmap_grid_map.h segmap_util.h segmap_particle_filter.h segmap_pose2d.h segmap_car_config.h \
	segmap_dataset.h segmap_viewer.h
PUBLIC_LIBRARIES = libsegmap.a
PUBLIC_BINARIES = 
TARGETS = libsegmap.a carmen_dataset_test

libsegmap.a: segmap_grid_map.o segmap_util.o segmap_particle_filter.o segmap_pose2d.o segmap_dataset.o segmap_viewer.o
carmen_dataset_test: carmen_dataset_test.o libsegmap.a

include $(CARMEN_HOME)/src/Makefile.rules
