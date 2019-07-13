include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = OUSTER
MODULE_COMMENT = Driver, and Visualizer for Ouster OS-1 64 Sensor

SUBDIRS += libouster_conn
LINK = g++ 

CXXFLAGS += -std=c++11 -Wno-deprecated
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11

# libraries required by the driver 
IFLAGS += -I /usr/include/jsoncpp/
LFLAGS += -ljsoncpp 

# viewer libraries
IFLAGS += -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ -I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`
LFLAGS += -L /usr/lib/x86_64-linux-gnu/ -lboost_system -lvtkIO -lvtkFiltering -lvtkCommon \
	 -lvtkRendering -lvtkGraphics -L /usr/local/lib -lpcl_common -lpcl_io -lpcl_visualization `pkg-config --libs opencv`

# carmen related libraries
LFLAGS += -lparam_interface -lglobal -lipc -lvelodyne_interface

SOURCES = ouster_main.cpp ouster_viewer_main.cpp

PUBLIC_INCLUDES = ouster_config.h
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = ouster ouster_viewer
TARGETS = ouster ouster_viewer

# The libsegmap.a is added as a dependency to make the program be compiled when a .cpp file in the lib is changed.
# Without adding the lib as a dependency, we would have to do a make clean && make whenever we change a .cpp file.
ouster: ouster_main.o libouster_conn/libouster_conn.a

ouster_viewer: ouster_viewer_main.o

include $(CARMEN_HOME)/src/Makefile.rules
