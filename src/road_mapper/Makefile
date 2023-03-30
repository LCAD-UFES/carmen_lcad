include ../Makefile.conf

MODULE_NAME = Road Mapper
MODULE_COMMENT = Create, view, learn and predict *road maps*

LINK = g++ -O3
CFLAGS += -Wall
CXXFLAGS += -Wno-ignored-qualifiers -O3

IFLAGS += \
	`pkg-config --cflags opencv` \

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -L/usr/local/carmen_boost/lib

LFLAGS += \
	-lparam_interface \
	-lglobal \
	-lipc \
	-lgrid_mapping \
	-lprob_models \
	-lz \
	-lrotation_geometry \
	-lmap_io \
	-lmap_util \
	-llocalize_ackerman_interface \
	-lmap_server_interface \
	-lmap_interface \
	-lpng \
	-lreadlog \
	-lkinect_interface \
	-lboost_signals \
	-lboost_system \
	`pkg-config --libs opencv` \

SOURCES = \
	road_mapper_main.cpp \
	road_mapper_interface.c \
	road_mapper_utils.cpp \
	road_mapper_save_remission_map_images.cpp \
	road_mapper_image_splitter.cpp \
	road_mapper_display_map.cpp \
	road_mapper_sampling.cpp \
	road_mapper_log_tracker.cpp \
	save_map_images.cpp \
	update_map_from_image.cpp \

PUBLIC_INCLUDES = \
	road_mapper_messages.h \
	road_mapper_interface.h \
	road_mapper.h \

PUBLIC_LIBRARIES = \
	libroad_mapper_interface.a \
	libroad_mapper.a \

PUBLIC_BINARIES = \
	road_mapper \
	road_mapper_save_remission_map_images \
	road_mapper_sampling \
	road_mapper_image_splitter \
	road_mapper_display_map \
	save_map_images \
	update_map_from_image \

PUBLIC_OTHERS = \
	rddf_generate.py \
	rddf_view.py \

TARGETS = \
	libroad_mapper_interface.a \
	libroad_mapper.a \
	libprob_models.a \
	road_mapper \
	road_mapper_save_remission_map_images \
	road_mapper_sampling \
	road_mapper_image_splitter \
	road_mapper_display_map \
	road_mapper_log_tracker \
	save_map_images \
	update_map_from_image \

ifneq (, $(shell which nvcc))
ifdef CAFFE_ENET_HOME

CXXFLAGS += -DUSE_CAFFE

IFLAGS += \
	-I /usr/local/cuda/include \
	-I$(CAFFE_ENET_HOME)/include \
	-I$(CAFFE_ENET_HOME)/build/include \

LFLAGS += \
	-L$(CAFFE_ENET_HOME)/build/lib \
	-lglog \
	-lprotobuf \
	-lcaffe \

SOURCES += \
	road_mapper_road_inference_main.cpp \

PUBLIC_BINARIES += \
	road_mapper_road_inference_main \

TARGETS += \
	road_mapper_road_inference_main \

endif
endif

# If you set graphics support in configure.
ifndef NO_GRAPHICS
#Carmen Global Graphic Library and Gtk+2 Graphics Libraries

IFLAGS += \
	`pkg-config --cflags gtk+-2.0 gmodule-export-2.0` \
	`pkg-config --cflags gtkglext-1.0` \

LFLAGS += \
	-lglobal_graphics \
	`pkg-config --libs gtk+-2.0 gmodule-export-2.0` \
	`pkg-config --libs gtkglext-1.0` \

endif

libroad_mapper_interface.a: road_mapper_interface.o
libroad_mapper.a: road_mapper.o
road_mapper: road_mapper_main.o road_mapper_interface.o
road_mapper_save_remission_map_images: road_mapper_save_remission_map_images.o road_mapper.o road_mapper_utils.o
road_mapper_sampling: road_mapper_sampling.o road_mapper.o road_mapper_utils.o
road_mapper_image_splitter: road_mapper_image_splitter.o
road_mapper_display_map: road_mapper_display_map.o road_mapper.o road_mapper_utils.o
road_mapper_log_tracker: road_mapper_log_tracker.o
save_map_images: save_map_images.o road_mapper.o road_mapper_utils.o
update_map_from_image: update_map_from_image.o road_mapper.o road_mapper_utils.o

libprob_models.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/prob_models/

clean: clean_prob_models

clean_prob_models:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/prob_models/

ifneq (, $(shell which nvcc))
ifdef CAFFE_ENET_HOME
road_mapper_road_inference_main: road_mapper_road_inference_main.o road_mapper.o road_mapper_utils.o
endif
endif
include ../Makefile.rules
