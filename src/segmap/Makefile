include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = SEG MAP MAIN
MODULE_COMMENT = Main programs

SUBDIRS += util libsegmap graphslam_fast gicp # scripts examples

LINK = g++ 

CXXFLAGS += -pg -fopenmp
LFLAGS += -pg
IFLAGS += -pg -fopenmp

CXXFLAGS += -Wno-deprecated -Wno-write-strings -Wno-unused-result -Wno-sign-compare -std=c++11
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11

IFLAGS += -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ \
	-I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv` -lopencv_imgcodecs
LFLAGS += -L /usr/lib/x86_64-linux-gnu/ -lboost_system
LFLAGS += -L /usr/lib -lvtkIO -lvtkFiltering -lvtkCommon -lvtkRendering -lvtkGraphics
LFLAGS += -L /usr/local/lib -lpcl_common -lpcl_io -lpcl_visualization -lglobal 
LFLAGS += -lboost_program_options -lboost_filesystem

LFLAGS += -lsegmap_types -lsegmap_datasets -lsegmap_readers -lsegmap_slam \
	-lsegmap_visualization -lsegmap_readers -lsegmap_motion_models \
	-lutil

DEPS = libsegmap/types/libsegmap_types.a \
	libsegmap/datasets/libsegmap_datasets.a \
	libsegmap/readers/libsegmap_readers.a \
	libsegmap/motion_models/libsegmap_motion_models.a \
	libsegmap/slam/libsegmap_slam.a \
	libsegmap/visualization/libsegmap_visualization.a \
	libsegmap/readers/libsemap_readers.a \
	util/libutil.a

SOURCES = mapper.cpp localizer.cpp  

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 
TARGETS = mapper localizer  

# The libraries are added as dependencies to make the program be compiled when the libs are changed.
# Without adding them as a dependency, we would have to do a make clean && make whenever we change a .cpp file.
localizer: localizer.o $(DEPS) 
mapper: mapper.o $(DEPS)

include $(CARMEN_HOME)/src/Makefile.rules
