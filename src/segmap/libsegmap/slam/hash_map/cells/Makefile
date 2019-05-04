
include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = LIB_SEG_MAP HASH MAP CELLS
MODULE_COMMENT = Cells that can be used for creating maps

LINK = g++ 

CXXFLAGS += -Wno-deprecated -Wno-write-strings -Wno-unused-result -Wno-sign-compare -std=c++11 
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11

IFLAGS += -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ \
	-I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`
	
SOURCES = segmap_occupancy_cell.cpp segmap_grayscale_cell.cpp segmap_color_cell.cpp segmap_height_cell.cpp
PUBLIC_INCLUDES = segmap_cell_interface.h segmap_occupancy_cell.h segmap_grayscale_cell.h segmap_color_cell.h \
	segmap_height_cell.h
PUBLIC_LIBRARIES = libsegmap_cells.a
PUBLIC_BINARIES = 
TARGETS = libsegmap_cells.a 

libsegmap_cells.a: segmap_occupancy_cell.o segmap_grayscale_cell.o segmap_color_cell.o segmap_height_cell.o 

include $(CARMEN_HOME)/src/Makefile.rules
