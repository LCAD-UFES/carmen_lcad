include ../Makefile.conf

MODULE_NAME = TRAFFIC_LIGHT 
MODULE_COMMENT = Mapping, Detection and Inference State of Traffic Lights

LINK = g++
CXXFLAGS += -std=c++0x

IFLAGS +=  -std=c++11 -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
IFLAGS += `pkg-config --cflags opencv`
LFLAGS += -lglobal -lparam_interface -lipc -lGL -lGLU -lglut \
	 -ltime_profile -lbumblebee_basic_interface -lstereo_interface -ltf \
	 -llocalize_ackerman_interface -lrddf_interface \
	  -L$(CARMEN_HOME)/sharedlib/libtf/src \
	 -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
	 -lboost_thread-mt -lrt -lboost_signals -lboost_system \
	  -L/usr/local/share/OpenCV/3rdparty/lib `pkg-config --libs opencv`

CFLAGS += -W -Wall 
CFLAGS += -O4 
CXXFLAGS += -std=c++0x

#SUBDIRS = traffic_light_view

SOURCES = traffic_light_main.cpp traffic_light_interface.cpp traffic_light.cpp mapping_traffic_light_main.cpp log_generate_images.cpp

PUBLIC_INCLUDES = traffic_light_interface.h traffic_light_messages.h traffic_light.h

PUBLIC_LIBRARIES = libtraffic_light_interface.a 

PUBLIC_BINARIES = traffic_light mapping_traffic_light log_generate_images

TARGETS = libtraffic_light_interface.a  traffic_light mapping_traffic_light log_generate_images

PUBLIC_LIBRARIES_SO = libtraffic_light_interface.so

ifndef NO_PYTHON
TARGETS += libtraffic_light_interface.so.1
endif

ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += -lparam_interface -lipc -lglobal -lglobal_graphics \
	`$(GTK_CONFIG) --libs` 
SOURCES += traffic_light_view.cpp
PUBLIC_BINARIES += traffic_light_view
TARGETS += traffic_light_view
endif

libtraffic_light_interface.a:		traffic_light_interface.o traffic_light.o

libtraffic_light_interface.so.1:	traffic_light_interface.o traffic_light.o

traffic_light:				traffic_light_main.o libtraffic_light_interface.a

mapping_traffic_light:			mapping_traffic_light_main.o libtraffic_light_interface.a

log_generate_images:			log_generate_images.o

traffic_light_view:			traffic_light_view.o libtraffic_light_interface.a

# rules
include ../Makefile.rules
