include ../Makefile.conf

# Module name and description
MODULE_NAME = MVOG_PACKAGE
MODULE_COMMENT = Multi Volumetric Occupancy Gridmap

LINK = g++

# Application specific include directories.
IFLAGS += -Imvog_model/include

# Required default libraries to comunicate with Carmen Core
LFLAGS += -L/usr/local/lib
LFLAGS += -lparam_interface -lipc -lglobal -lkinect_interface -lfused_odometry_interface -llaser_interface \
	  -lstereo_interface -lmvog_interface -lboost_system

IFLAGS += `pkg-config --cflags bullet`
LFLAGS += `pkg-config --libs bullet`
IFLAGS += `pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv`

CFLAGS += -Wall
# Source code files (.c, .cpp) 
SOURCES = mvog_main.cpp mvog_interface.c
SOURCES += mvog_model/src/cell.cpp mvog_model/src/cell_array.cpp mvog_model/src/cell_vector.cpp  \
mvog_model/src/map.cpp mvog_model/src/mapper.cpp mvog_model/src/volume.cpp mvog_model/src/volume_vector.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = mvog_messages.h mvog_interface.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libmvog_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = mvog

TARGETS = libmvog_interface.a mvog libmvog_model.a
PUBLIC_LIBRARIES_SO = libmvog_interface.so

ifndef NO_PYTHON
TARGETS += libmvog_interface.so.1
endif

# If you set graphics support in configure.
ifndef NO_GRAPHICS
IFLAGS += -Imvog_view/include
IFLAGS += -I/usr/include/libglade-2.0/
IFLAGS += `pkg-config --cflags gtk+-2.0 gmodule-export-2.0`
IFLAGS += `pkg-config --cflags gtkglext-1.0`
LFLAGS += `pkg-config --libs gtk+-2.0 gmodule-export-2.0`
LFLAGS += `pkg-config --libs gtkglext-1.0`

#Carmen Global Graphic Library and Gtk+2 Graphics Libraries
LFLAGS += -lglobal_graphics -lgdk-x11-2.0 -lgdk_pixbuf-2.0
#Source code file (.c)
SOURCES += mvog_view/src/callbacks.cpp mvog_view/src/camera.cpp mvog_view/src/draw_callbacks.cpp \
mvog_view/src/gtk_gui.cpp mvog_view/src/map_drawer_2d.cpp mvog_view/src/map_drawer_3d.cpp
PUBLIC_BINARIES +=
GLADE_FILES = gui.glade
TARGETS += libmvog_view.a
endif

clean: model_clean view_clean

model_clean: 
#	$(MAKE) -C mvog_model/src clean
	-rm -rf mvog_model/src/*.o

view_clean: 
#	$(MAKE) -C mvog_view/src clean
	-rm -rf mvog_view/src/*.o

libmvog_interface.a:	mvog_interface.o

libmvog_interface.so.1:	mvog_interface.o

libmvog_model.a: mvog_model/src/cell.o mvog_model/src/cell_array.o mvog_model/src/cell_vector.o \
mvog_model/src/map.o mvog_model/src/mapper.o  mvog_model/src/volume.o mvog_model/src/volume_vector.o

libmvog_view.a: mvog_view/src/callbacks.o mvog_view/src/camera.o mvog_view/src/draw_callbacks.o \
mvog_view/src/gtk_gui.o mvog_view/src/map_drawer_2d.o mvog_view/src/map_drawer_3d.o

mvog: mvog_main.o libmvog_interface.a libmvog_model.a libmvog_view.a 

# rules

include ../Makefile.rules
