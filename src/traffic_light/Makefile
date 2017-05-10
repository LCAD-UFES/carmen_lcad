include ../Makefile.conf

MODULE_NAME = TRAFFIC_LIGHT 
MODULE_COMMENT = Mapping, Detection and Inference State of Traffic Lights

LINK = g++
CXXFLAGS += -std=c++0x -Wno-ignored-qualifiers# -g -pg
CFLAGS += -Wall # -g -pg
SUBDIRS += TLightStateRecog

IFLAGS += -I $(CAFFE_HOME)/include -I $(CAFFE_HOME)/build/src/ -I /usr/local/cuda/include `pkg-config --cflags opencv`
IFLAGS +=  -std=c++11 -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/

LFLAGS += -L $(CAFFE_HOME)/build/lib -lcaffe \
		-L /usr/local/cuda/lib64 -lglog -lprotobuf -L/usr/local/lib

LFLAGS += -lglobal -lparam_interface -lipc -lGL -lGLU -lglut \
	 -ltime_profile -lbumblebee_basic_interface -lstereo_interface -ltf \
	 -llocalize_ackerman_interface -lrddf_interface \
	  -L$(CARMEN_HOME)/sharedlib/libtf/src \
	 -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
	 -lboost_thread-mt -lrt -lboost_signals -lboost_system \
	  -L/usr/local/share/OpenCV/3rdparty/lib `pkg-config --libs opencv` \
	  -lwnn -ldlib -lblas

#SOURCES = traffic_light_main.cpp traffic_light_interface.cpp traffic_light.cpp mapping_traffic_light_main.cpp log_generate_images.cpp
SOURCES = traffic_light_main.cpp traffic_light_interface.cpp traffic_light.cpp log_generate_images.cpp squeezenet_tlight.cpp #tlight_train.cpp tlight_evaluate_train.cpp 

PUBLIC_INCLUDES = traffic_light_interface.h traffic_light_messages.h traffic_light.h squeezenet_tlight.h

PUBLIC_LIBRARIES = libtraffic_light_interface.a libsqueezenet_tlight.a

#PUBLIC_BINARIES = traffic_light mapping_traffic_light log_generate_images
PUBLIC_BINARIES = traffic_light log_generate_images

#TARGETS = libtraffic_light_interface.a  traffic_light mapping_traffic_light log_generate_images
TARGETS = libtraffic_light_interface.a  traffic_light log_generate_images libsqueezenet_tlight.a #tlight_train tlight_evaluate_train

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

#tlight_evaluate_train: tlight_evaluate_train.o tlight_vgram.o
#tlight_train: tlight_train.o tlight_vgram.o

libsqueezenet_tlight.a: squeezenet_tlight.o

libtraffic_light_interface.a:		traffic_light_interface.o traffic_light.o

libtraffic_light_interface.so.1:	traffic_light_interface.o traffic_light.o

SOURCES += TLightStateRecog/tlight_vgram.cpp
traffic_light: traffic_light_main.o libtraffic_light_interface.a TLightStateRecog/tlight_vgram.o squeezenet_tlight.o #tlight_vgram.o 

#mapping_traffic_light:			mapping_traffic_light_main.o libtraffic_light_interface.a

log_generate_images:			log_generate_images.o

traffic_light_view:			traffic_light_view.o libtraffic_light_interface.a

# rules
include ../Makefile.rules
