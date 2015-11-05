include ../../Makefile.conf

MODULE_NAME = LIBTLD
MODULE_COMMENT = TLD Library

LINK = g++

IFLAGS += -I./include
IFLAGS += -I$(OPENCV_HOME)/include
LFLAGS += -L$(OPENCV_HOME)/lib
LFLAGS += -lopencv_core -lopencv_imgproc -lopencv_video

SOURCES = ./src/Detector.cpp \
    ./src/Utils.cpp \
    ./src/Rect.cpp \
    ./src/Fern.cpp \
    ./src/Lk.cpp

TARGETS = libtld.a

libtld.a: ./src/Detector.o \
    ./src/Utils.o \
    ./src/Rect.o \
    ./src/Fern.o \
    ./src/Lk.o

# rules

include ../../Makefile.rules