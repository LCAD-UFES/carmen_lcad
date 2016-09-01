include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CFLAGS += -pg
CXXFLAGS += -std=c++0x -pg 

MODULE_NAME = Car Neural Model
MODULE_COMMENT = Car Neural Ackeman Motion Model

#IFLAGS +=
LFLAGS += -lfann

SOURCES = car_neural_model.cpp
 
PUBLIC_INCLUDES = car_neural_model.h
PUBLIC_LIBRARIES = libcarneuralmodel.a
#PUBLIC_BINARIES =

TARGETS = libcarneuralmodel.a  
PUBLIC_LIBRARIES_SO =

libcarneuralmodel.a: car_neural_model.o

include $(CARMEN_HOME)/src/Makefile.rules