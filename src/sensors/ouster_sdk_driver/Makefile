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

LFLAGS += -lparam_interface -lglobal -lipc -pthread -lvelodyne_interface -louster_client -louster_viz -louster_pcap -ltins -lpcap
LFLAGS += -L /usr/lib/x86_64-linux-gnu/ -lboost_system

# carmen related libraries

SOURCES = ouster_main.cpp types.cpp helpers.cpp split_vertical_angles_list.cpp # ouster_log_raw_data.cpp ouster_read_pcad_file.cpp ouster_publish_from_pcap.cpp types.cpp helpers.cpp # ouster_viewer_main.cpp

PUBLIC_INCLUDES =
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = ouster # ouster_viewer
TARGETS = ouster split_vertical_angles_list generate_parameters_list#ouster_log_raw_data ouster_read_pcad_file ouster_publish_from_pcap # ouster_viewer

# The libsegmap.a is added as a dependency to make the program be compiled when a .cpp file in the lib is changed.
# Without adding the lib as a dependency, we would have to do a make clean && make whenever we change a .cpp file.
ouster: ouster_main.o types.o
ouster_log_raw_data: ouster_log_raw_data.o types.o
ouster_read_pcad_file: ouster_read_pcad_file.o helpers.o types.o 
ouster_publish_from_pcap: ouster_publish_from_pcap.o helpers.o types.o 
split_vertical_angles_list: split_vertical_angles_list.o types.o
generate_parameters_list: generate_parameters_list.o types.o

include $(CARMEN_HOME)/src/Makefile.rules
