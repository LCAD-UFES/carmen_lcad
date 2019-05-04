
include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = LIB_SEG_MAP HASH MAP
MODULE_COMMENT = Hash grid maps

LINK = g++ 
SUBDIRS += distributions cells

CXXFLAGS += -Wno-deprecated -Wno-write-strings -Wno-unused-result -Wno-sign-compare -std=c++11 
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11

IFLAGS += -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ \
	-I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`

SOURCES = 
PUBLIC_INCLUDES = segmap_hash_map.h
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 
TARGETS = 

include $(CARMEN_HOME)/src/Makefile.rules
