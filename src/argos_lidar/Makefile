include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = ARGOS_LIDAR
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

# ROS Geral
ROS2_PREFIX = /opt/ros/foxy

IFLAGS += -I$(ROS2_PREFIX)/include -I. -I./unitree_go -I./unitree_go/msg -I./unitree_go/msg/detail
IFLAGS += -I$(ROS2_PREFIX)/include/rclcpp

LFLAGS += -L$(ROS2_PREFIX)/lib -L/home/unitree/I2CA/argos/install/unitree_go/lib/
LFLAGS += -lunitree_go__rosidl_typesupport_cpp -llibstatistics_collector -lrcl -ltracetools -lstatistics_msgs__rosidl_typesupport_cpp \
			-lrclcpp -lrcutils -lrosidl_runtime_c -lrosidl_typesupport_cpp -lrosidl_typesupport_c -lstd_msgs__rosidl_typesupport_cpp 

#

# carmen related libraries

SOURCES = ouster_main.cpp types.cpp 

PUBLIC_INCLUDES =
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = argos_lidar # ouster_viewer
TARGETS = argos_lidar 

# The libsegmap.a is added as a dependency to make the program be compiled when a .cpp file in the lib is changed.
# Without adding the lib as a dependency, we would have to do a make clean && make whenever we change a .cpp file.
argos_lidar: ouster_main.o types.o

include $(CARMEN_HOME)/src/Makefile.rules
