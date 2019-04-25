
include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = LIB_SEG_MAP DATASETS
MODULE_COMMENT = Classes for reading datasets

LINK = g++ 

CXXFLAGS += -Wno-deprecated -Wno-write-strings -Wno-unused-result -Wno-sign-compare -std=c++11 
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11

IFLAGS += -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ -I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`

SOURCES = segmap_dataset.cpp # segmap_dataset_old.cpp  
PUBLIC_INCLUDES = segmap_dataset.h segmap_car_config.h # segmap_dataset_old.h 
PUBLIC_LIBRARIES = libsegmap_datasets.a
PUBLIC_BINARIES = 
TARGETS = libsegmap_datasets.a

libsegmap_datasets.a: segmap_dataset.o # segmap_dataset_old.o  

include $(CARMEN_HOME)/src/Makefile.rules
