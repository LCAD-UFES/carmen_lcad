include ../Makefile.conf

CC = g++
LINK = g++
CFLAGS += -std=c99

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models

IFLAGS += `pkg-config --cflags opencv`
LFLAGS += -L/usr/local/lib
LFLAGS += -lglobal -lparam_interface -lipc -lprob_models \
		-ltime_profile -lbumblebee_basic_interface -lmap_interface -lroad_finding  -lgsl \
		-lGL -lGLU -lglut -lm -lgslcblas -fopenmp \
		-lstereo_interface -lxsens_interface -lvisual_odometry_interface -lfused_odometry_interface \
		-lv_disparity -fopenmp `pkg-config --libs opencv` -lrotation_geometry
			
MODULE_NAME = STEREO_MAPPING
MODULE_COMMENT = Generate a Map based on Computer Vision 

SOURCES = stereo_mapping.cpp stereo_mapping_interface.c  stereo_mapping_map.c stereo_mapping_kalman_filter.cpp
PUBLIC_INCLUDES = stereo_mapping_messages.h stereo_mapping_interface.h stereo_mapping_map.h stereo_mapping_kalman_filter.h

PUBLIC_LIBRARIES = libstereo_mapping_interface.a
PUBLIC_BINARIES = stereo_mapping
MAN_PAGES =

TARGETS = libprob_models.a stereo_mapping libstereo_mapping_interface.a

PUBLIC_LIBRARIES_SO = 

ifndef NO_PYTHON
TARGETS += 
endif

ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += -lparam_interface -lipc -lglobal -lglobal_graphics \
	`$(GTK_CONFIG) --libs` 
SOURCES += stereo_mapping_view.cpp
PUBLIC_BINARIES += stereo_mapping_view
TARGETS += stereo_mapping_view
endif

libstereo_mapping_interface.a: stereo_mapping_interface.o

stereo_mapping: stereo_mapping.o stereo_mapping_map.o stereo_mapping_kalman_filter.o libstereo_mapping_interface.a
 
stereo_mapping_view: stereo_mapping_view.o libstereo_mapping_interface.a

# rules

libprob_models.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/prob_models/

clean: clean_prob_models

clean_prob_models:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/prob_models/
	
include ../Makefile.rules

