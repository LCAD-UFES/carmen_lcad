include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = LIB_SEG_MAP VISUALIZATION
MODULE_COMMENT = Visualization functions

LINK = g++ 

CXXFLAGS += -Wno-deprecated -Wno-write-strings -Wno-unused-result -Wno-sign-compare -std=c++11 

# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11
IFLAGS += -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ -I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`
LFLAGS += -L /usr/local/lib -lpcl_common -lpcl_io -lpcl_visualization

SOURCES = segmap_semantic_segmentation_viewer.cpp segmap_sensor_viewer.cpp \
	segmap_printers.cpp segmap_particle_filter_viewer.cpp

PUBLIC_INCLUDES = segmap_semantic_segmentation_viewer.h segmap_sensor_viewer.h \
	segmap_colormaps.h segmap_printers.h segmap_particle_filter_viewer.h 
PUBLIC_LIBRARIES = libsegmap_visualization.a
PUBLIC_BINARIES = 
TARGETS = libsegmap_visualization.a

libsegmap_visualization.a: segmap_semantic_segmentation_viewer.o segmap_sensor_viewer.o \
	segmap_printers.o segmap_particle_filter_viewer.o

include $(CARMEN_HOME)/src/Makefile.rules
