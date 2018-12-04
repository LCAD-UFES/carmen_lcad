include ../Makefile.conf

MODULE_NAME = SEG MAP
MODULE_COMMENT = SLAM using semantic segmentation

SUBDIRS += graphslam_fast libsegmap scripts

LINK = g++ 

CXXFLAGS += -pg
LFLAGS += -pg
IFLAGS += -pg

CXXFLAGS += -Wno-deprecated -Wno-write-strings -Wno-unused-result -Wno-sign-compare -std=c++11
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11

IFLAGS += -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ -I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv` -lsegmap -lopencv_imgcodecs
LFLAGS += -L /usr/lib/x86_64-linux-gnu/ -lboost_system
LFLAGS += -L /usr/lib -lvtkIO -lvtkFiltering -lvtkCommon -lvtkRendering -lvtkGraphics
LFLAGS += -L /usr/local/lib -lpcl_common -lpcl_io -lpcl_visualization

SOURCES = localizer.cpp mapper.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 
TARGETS = localizer mapper 

# The libsegmap.a is added as a dependency to make the program be compiled when a .cpp file in the lib is changed.
# Without adding the lib as a dependency, we would have to do a make clean && make whenever we change a .cpp file.
localizer: localizer.o libsegmap/libsegmap.a
mapper: mapper.o libsegmap/libsegmap.a

include ../Makefile.rules
