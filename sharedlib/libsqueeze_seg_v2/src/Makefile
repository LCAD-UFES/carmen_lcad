include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CFLAGS += 
CXXFLAGS += -std=c++11

MODULE_NAME = SqueezeSeg V2 Neural Network
MODULE_COMMENT = Semantic segmentation of LiDAR point cloud

IFLAGS += -I/usr/include/python2.7
LFLAGS += -lpython2.7 -lpthread `pkg-config --libs opencv` `python2.7-config --libs`

SOURCES = libsqueeze_seg.cpp
 
PUBLIC_INCLUDES = libsqueeze_seg.h
PUBLIC_LIBRARIES = libsqueeze_seg.a
#PUBLIC_BINARIES =

TARGETS = libsqueeze_seg.a
PUBLIC_LIBRARIES_SO =

libsqueeze_seg.a: libsqueeze_seg.o


# https://stackoverflow.com/questions/7369145/activating-a-virtualenv-using-a-shell-script-doesnt-seem-to-work
virtualenv:
	. create_env.sh


include $(CARMEN_HOME)/src/Makefile.rules