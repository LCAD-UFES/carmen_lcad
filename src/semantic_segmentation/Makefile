
include ../Makefile.conf

# Module name and description
MODULE_NAME = SEMANTIC_SEGMENTATION
MODULE_COMMENT = Semantic Segmentation

LINK = g++

IFLAGS += -I $(CAFFE_HOME)/include -I $(CAFFE_HOME)/build/src/ -I /usr/local/cuda/include `pkg-config --cflags opencv` 
LFLAGS += -lparam_interface -lipc -lglobal -lbumblebee_basic_interface -L $(CAFFE_HOME)/build/lib -lcaffe \
	-L /usr/local/cuda/lib64 `pkg-config --libs opencv` -lglog -lprotobuf -L/usr/local/lib -lboost_thread -lboost_system
CFLAGS += -Wall 
CXXFLAGS += -Wall -std=c++0x

SOURCES = semantic_segmentation_main.cpp semantic_segmentation_interface.cpp semantic_segmentation.cpp semantic_segmentation_test.cpp semantic_segmentation_all_images.cpp

PUBLIC_INCLUDES = semantic_segmentation_messages.h semantic_segmentation_interface.h semantic_segmentation.h
PUBLIC_LIBRARIES = libsemantic_segmentation_interface.a libsemantic_segmentation.a
PUBLIC_BINARIES = semantic_segmentation_main semantic_segmentation_test semantic_segmentation_all_images

TARGETS = libsemantic_segmentation_interface.a libsemantic_segmentation.a semantic_segmentation_main semantic_segmentation_test semantic_segmentation_all_images

libsemantic_segmentation_interface.a: semantic_segmentation_interface.o 
libsemantic_segmentation.a: semantic_segmentation.o
semantic_segmentation_main:	semantic_segmentation_main.o semantic_segmentation.o semantic_segmentation_interface.o
semantic_segmentation_test: semantic_segmentation_test.o semantic_segmentation.o
semantic_segmentation_all_images: semantic_segmentation_all_images.o semantic_segmentation.o

include ../Makefile.rules

