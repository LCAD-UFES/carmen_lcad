include ../Makefile.conf

MODULE_NAME = ARGOS LIDAR
MODULE_COMMENT = "Argos Lidar"

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol -Wno-switch
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcontrol


# ROS
IFLAGS += -I/opt/ros/foxy/include
LFLAGS += -L/opt/ros/foxy/lib

LFLAGS += -lglobal -lipc -lparam_interface -llocalize_ackerman_interface -lrddf_interface -lroute_planner_interface \
		  -lnavigator_ackerman_interface -lbehavior_selector_interface -lford_escape_hybrid_interface -lcargo_interface \
		  -lvoice_interface_interface -luser_app_server_interface \
		  -lrclcpp -lrcutils -pthread -lrmw -lrcl -lrosidl_typesupport_cpp -llibstatistics_collector -lvelodyne_interface \
		  -lsensor_msgs__python \
		  -lsensor_msgs__rosidl_generator_c \
		  -lsensor_msgs__rosidl_typesupport_connext_cpp \
		  -lsensor_msgs__rosidl_typesupport_connext_c \
		  -lsensor_msgs__rosidl_typesupport_cpp \
		  -lsensor_msgs__rosidl_typesupport_c \
		  -lsensor_msgs__rosidl_typesupport_fastrtps_cpp \
		  -lsensor_msgs__rosidl_typesupport_fastrtps_c \
		  -lsensor_msgs__rosidl_typesupport_introspection_cpp \
		  -lsensor_msgs__rosidl_typesupport_introspection_c \
		  -lstatistics_msgs__python \
		  -lstatistics_msgs__rosidl_generator_c \
		  -lstatistics_msgs__rosidl_typesupport_connext_cpp \
		  -lstatistics_msgs__rosidl_typesupport_connext_c \
		  -lstatistics_msgs__rosidl_typesupport_cpp \
		  -lstatistics_msgs__rosidl_typesupport_c \
		  -lstatistics_msgs__rosidl_typesupport_fastrtps_cpp \
		  -lstatistics_msgs__rosidl_typesupport_fastrtps_c \
		  -lstatistics_msgs__rosidl_typesupport_introspection_cpp \
		  -lstatistics_msgs__rosidl_typesupport_introspection_c \
		  -ltracetools

CXXFLAGS += -std=c++14  

SOURCES = argos_lidar_main.cpp argos_lidar_luiz.cpp argos_lidar_polar_main.cpp argos_lidar_viewer_main.cpp
TARGETS = argos_lidar argos_lidar_luiz argos_lidar_polar argos_lidar_viewer
PUBLIC_BINARIES = argos_lidar argos_lidar_luiz argos_lidar_polar argos_lidar_viewer

argos_lidar: argos_lidar_main.o  
argos_lidar_luiz: argos_lidar_luiz.o
argos_lidar_polar: argos_lidar_polar_main.o
argos_lidar_viewer: argos_lidar_viewer_main.o

include ../Makefile.rules
