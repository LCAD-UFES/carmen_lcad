include ../Makefile.conf

# Module name and description
MODULE_NAME = GRID_MAPPING_PACKAGE
MODULE_COMMENT = Grid Mapping Package

LINK = g++

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L/usr/local/carmen_boost/lib

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lprob_models -lmap_io -lglobal -lparam_interface -llaser_interface -lipc \
	-lobstacle_avoider_interface -llocalize_ackerman_interface \
	-lultrasonic_filter_interface -lvisual_odometry_interface -ljpeg \
	-llocalize_ackerman_interface -lmap_interface  -lmap_server_interface \
	-ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
	-lboost_thread-mt -lrt -lboost_signals -lboost_system -lrotation_geometry \
	`pkg-config --libs opencv`
	
CFLAGS +=  -std=c99

# Source code files (.c, .cpp) 

SOURCES = message_interpolation.cpp grid_mapping.c build_complete_map.c complete_map_to_block_map.c build_road_map.cpp
#MODIFICACOES DE LUAN:
#SOURCES = grid_mapping_main.cpp message_interpolation.cpp grid_mapping_interface.c  grid_mapping.c grid_mapping_test.c build_complete_map.c complete_map_to_block_map.c complete_map_to_bitmap.c bitmap_to_complete_map.c

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = grid_mapping.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libgrid_mapping.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = grid_mapping build_complete_map complete_map_to_block_map complete_map_to_bitmap bitmap_to_complete_map build_average_map build_road_map

TARGETS = libprob_models.a libgrid_mapping.a  build_complete_map complete_map_to_block_map complete_map_to_bitmap bitmap_to_complete_map build_average_map build_road_map
#MODIFICACOES DE LUAN:
#TARGETS = libprob_models.a libgrid_mapping_interface.a libgrid_mapping.a grid_mapping  grid_mapping_test build_complete_map complete_map_to_block_map complete_map_to_bitmap bitmap_to_complete_map

PUBLIC_LIBRARIES_SO =

ifndef NO_PYTHON
TARGETS += 
endif

# If you set graphics support in configure.
ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += `pkg-config --libs gtk+-2.0`

#Carmen Global Graphic Library and Gtk+2 Graphics Libraries
LFLAGS += -lglobal_graphics -lglib-2.0

#Source code file (.c)
endif

libgrid_mapping.a: grid_mapping.o

#grid_mapping: grid_mapping_main.o libgrid_mapping.a

build_complete_map: build_complete_map.o libgrid_mapping.a

complete_map_to_block_map: complete_map_to_block_map.o libgrid_mapping.a

complete_map_to_bitmap: complete_map_to_bitmap.o libgrid_mapping.a

bitmap_to_complete_map: bitmap_to_complete_map.o libgrid_mapping.a

build_average_map: build_average_map.o libgrid_mapping.a

build_road_map: build_road_map.o libgrid_mapping.a

# rules

libprob_models.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/prob_models/

clean: clean_prob_models

clean_prob_models:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/prob_models/
	
include ../Makefile.rules
