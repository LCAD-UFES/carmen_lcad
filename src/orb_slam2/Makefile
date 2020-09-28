
include ../Makefile.conf

MODULE_NAME = ORB_SLAM2
MODULE_COMMENT = SLAM based on Monocular or Stereo Camera

LINK = g++ 
CXXFLAGS += -Wno-deprecated -Wno-deprecated-declarations

# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11
CFLAGS += -std=c11
CXXFLAGS += -std=c++11

LFLAGS += -lglobal -lipc -lbumblebee_basic_interface -lparam_interface 

# ORB_SLAM2 Dependencies
IFLAGS += -I/usr/include/suitesparse -I/usr/include/eigen3 `pkg-config --cflags opencv` 
LFLAGS += -lGL -lGLEW -lpangolin `pkg-config --libs opencv`


# ORB_SLAM2
IFLAGS += -I ~/packages_carmen/ORB_SLAM2/include -I ~/packages_carmen/ORB_SLAM2/
LFLAGS += -L ~/packages_carmen/ORB_SLAM2/lib -L ~/packages_carmen/ORB_SLAM2/Thirdparty/DBoW2/lib 
LFLAGS += ~/packages_carmen/ORB_SLAM2/Thirdparty/g2o/lib/libg2o.so
LFLAGS += -lORB_SLAM2 -lDBoW2

SOURCES = orb_slam2_main.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = orb_slam2_main
TARGETS = orb_slam2_main

orb_slam2_main: orb_slam2_main.o

include ../Makefile.rules

