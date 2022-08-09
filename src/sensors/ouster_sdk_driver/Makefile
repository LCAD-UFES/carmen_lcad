include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = OUSTER
MODULE_COMMENT = Driver, and Visualizer for Ousters Sensors

LINK = g++

CXXFLAGS += -std=c++17 -Wno-deprecated -Wno-deprecated-declarations
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++17

# libraries required by the driver 
IFLAGS += -I /usr/include/jsoncpp/ -I /usr/local/lib/
LFLAGS += -ljsoncpp 

# viewer libraries
IFLAGS += -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ -I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`
IFLAGS += -I /usr/local/include/ouster -I /usr/local/include/optional-lite -I /usr/local/include/ouster/imp

LFLAGS += -lparam_interface -lglobal -lipc -pthread -lvelodyne_interface -louster_client -louster_viz -louster_pcap
LFLAGS += -L /usr/lib/x86_64-linux-gnu/ -lboost_system

# carmen related libraries

SOURCES = ouster_main.cpp # ouster_viewer_main.cpp

PUBLIC_INCLUDES =
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = ouster2 # ouster_viewer
TARGETS = ouster2 # ouster_viewer

# The libsegmap.a is added as a dependency to make the program be compiled when a .cpp file in the lib is changed.
# Without adding the lib as a dependency, we would have to do a make clean && make whenever we change a .cpp file.
ouster2: ouster_main.o types.o

include $(CARMEN_HOME)/src/Makefile.rules
