include ../Makefile.conf

MODULE_NAME = ARGOS_GLOBALPOS
MODULE_COMMENT = "Argos GlobalPos"

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol -Wno-switch
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcontrol


IFLAGS += -I/opt/ros/foxy/include
LFLAGS += -L/opt/ros/foxy/lib

LFLAGS += -lglobal -lipc -lparam_interface -llocalize_ackerman_interface -lrddf_interface \
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
		  -ltracetools\

CXXFLAGS += -std=c++14  

SOURCES = argos_globalpos_main.cpp 
TARGETS = argos_globalpos 
PUBLIC_BINARIES = argos_globalpos

argos_globalpos: argos_globalpos_main.o  


include ../Makefile.rules
