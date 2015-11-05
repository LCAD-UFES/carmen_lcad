include ../Makefile.conf

LINK = g++

MODULE_NAME = CARMEN_ROS_INTERFACE 
MODULE_COMMENT = Module to provide communication between ros and carmen

IFLAGS += `pkg-config --cflags opencv`

LFLAGS += -lstereo_interface -lparam_interface -lipc -lglobal -lbumblebee_basic_interface \
			-lvisual_search_thin_interface -lvergence_interface   

LFLAGS += `pkg-config --libs opencv`

## ROS CORE
IFLAGS += -I /opt/ros/fuerte/include/
LFLAGS += -L /opt/ros/fuerte/lib -lroscpp -lroslib -lcpp_common -lrostime -lrosconsole \
	-lroscpp_serialization
## END

SOURCES = bumblebee_publisher.cpp visual_search_thin_communication.cpp
 
PUBLIC_INCLUDES =
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = bumblebee_publisher visual_search_thin_communication

bumblebee_publisher: bumblebee_publisher.o

visual_search_thin_communication: visual_search_thin_communication.o

TARGETS = bumblebee_publisher visual_search_thin_communication
				 
# rules
include ../Makefile.rules
