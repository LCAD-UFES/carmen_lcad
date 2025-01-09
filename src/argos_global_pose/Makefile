include ../Makefile.conf

MODULE_NAME = ARGOS GLOBAL POSE
MODULE_COMMENT = "Argos Global Pose"

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol -Wno-switch
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcontrol


# ROS
IFLAGS += -I/opt/ros/foxy/include
LFLAGS += -L/opt/ros/foxy/lib

LFLAGS += -lglobal -lipc -lparam_interface -llocalize_ackerman_interface -lrddf_interface -lroute_planner_interface \
		  -lnavigator_ackerman_interface -lbehavior_selector_interface -lford_escape_hybrid_interface -lcargo_interface \
		  -lvoice_interface_interface -luser_app_server_interface \
		  -lrclcpp -lrcutils -pthread -lrmw -lrcl -lrosidl_typesupport_cpp -llibstatistics_collector -lvelodyne_interface \
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
		  -lrosidl_typesupport_cpp \
		  -lrosidl_typesupport_c \
          -lrosidl_runtime_c \
		  -lstdc++ \
          -lgeometry_msgs__rosidl_typesupport_fastrtps_cpp \
          -lgeometry_msgs__rosidl_typesupport_introspection_cpp\
		  -lgeometry_msgs__rosidl_typesupport_fastrtps_c \
          -lgeometry_msgs__rosidl_typesupport_introspection_c\
		  -lgeometry_msgs__rosidl_typesupport_cpp \
		  -lrosidl_typesupport_cpp \
          -lrosidl_runtime_c \
		  -ltracetools\
		  -ltf2\

CXXFLAGS += -std=c++14  

SOURCES = argos_global_pose_main.cpp 
TARGETS = argos_global_pose 
PUBLIC_BINARIES = argos_global_pose

argos_global_pose: argos_global_pose_main.o  



include ../Makefile.rules
