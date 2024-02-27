include $(CARMEN_HOME)/src/Makefile.conf

# Module name and description
MODULE_NAME = TF Carmen Library
MODULE_COMMENT = Transformation Coordinate Library

LINK = g++

CFLAGS += -std=c11 -DBOOST_SIGNALS_NO_DEPRECATION_WARNING
CXXFLAGS += -std=c++11 -DBOOST_SIGNALS_NO_DEPRECATION_WARNING -Wno-parentheses -Wno-deprecated-copy -Wno-implicit-fallthrough -Wno-deprecated-declarations
# Application specific include directories.
IFLAGS += -I/usr/include/bullet/ -I/usr/local/include/bullet/ -I/usr/include/boost
IFLAGS += -Iinclude/tf/

# Required default libraries to comunicate with Carmen Core
LFLAGS += -L/usr/lib/x86_64-linux-gnu
LFLAGS += -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lrt -lboost_system
CFLAGS += -Wall
# Source code files (.c, .cpp) 
SOURCES = tf_rate.cpp tf_duration.cpp tf_time.cpp tf.cpp cache.cpp main.cpp  

PUBLIC_INCLUDES = tf.h

PUBLIC_LIBRARIES = libtf.a

TARGETS = libtf.a test

libtf.a: tf_rate.o tf_duration.o tf_time.o tf.o cache.o 

test: main.o libtf.a

# rules

include $(CARMEN_HOME)/src/Makefile.rules
