include ../Makefile.conf

MODULE_NAME = ARGOS_ROS_MAPPER
MODULE_COMMENT = "Argos ROS Mapper"

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol -Wno-switch
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcontrol


IFLAGS += -I/opt/ros/foxy/include
LFLAGS += -L/opt/ros/foxy/lib 

LFLAGS += -lglobal -lipc -lparam_interface -lgrid_mapping -lmapper -lmap_io -lmap_util -lmap_interface -lmapper_interface \
		  -lrclcpp -lrcutils -pthread -lrmw -lrcl \
		  -lrosidl_typesupport_cpp \
		  -llibstatistics_collector  \
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
          -lgeometry_msgs__rosidl_typesupport_fastrtps_cpp \
          -lgeometry_msgs__rosidl_typesupport_introspection_cpp\
		  -lgeometry_msgs__rosidl_typesupport_fastrtps_c \
          -lgeometry_msgs__rosidl_typesupport_introspection_c\
		  -lgeometry_msgs__rosidl_typesupport_cpp \
	      -lrosidl_typesupport_cpp \
		  -lrosidl_typesupport_c \
          -lrosidl_runtime_c \
		  -lnav_msgs__rosidl_typesupport_cpp\
		  -ltracetools

CXXFLAGS += -std=c++14  

SOURCES = argos_ros_mapper_main.cpp 
TARGETS = argos_ros_mapper 
PUBLIC_BINARIES = argos_ros_mapper

argos_ros_mapper: argos_ros_mapper_main.o  
	bash -c "source /opt/ros/foxy/setup.bash && $(CXX) $(CXXFLAGS) $(IFLAGS) $^ -o $@ $(LFLAGS)"


include ../Makefile.rules
