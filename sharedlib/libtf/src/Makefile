include $(CARMEN_HOME)/src/Makefile.conf

# Module name and description
MODULE_NAME = TF Carmen Library
MODULE_COMMENT = Transformation Coordinate Library

LINK = g++

CFLAGS += -std=c11 -DBOOST_SIGNALS_NO_DEPRECATION_WARNING
CXXFLAGS += -std=c++11 -DBOOST_SIGNALS_NO_DEPRECATION_WARNING
# Application specific include directories.
IFLAGS += -I/usr/local/include/bullet/ -I/usr/include/bullet/
IFLAGS += -Iinclude/tf/

# Required default libraries to comunicate with Carmen Core
LFLAGS += -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system
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
