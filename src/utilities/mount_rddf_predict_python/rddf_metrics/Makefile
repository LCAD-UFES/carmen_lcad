include ../../../Makefile.conf

LINK = g++
CFLAGS += 
LFLAGS += -lglobal -lipc -lparam_interface -llocalize_ackerman_interface -lbumblebee_basic_interface -lgsl -lgslcblas

#G2O
IFLAGS += -I/usr/include/eigen3 
IFLAGS += -I/usr/include/suitesparse -I/usr/include/eigen3 -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/

MODULE_NAME = RDDF_METRICS
MODULE_COMMENT = 

SOURCES = rddf_metrics.cpp rddf_predict_optimizer.cpp
PUBLIC_INCLUDES = rddf_predict_optimizer.h
PUBLIC_LIBRARIES = rddf_predict_optimizer.a
PUBLIC_BINARIES = rddf_metrics

TARGETS = rddf_metrics rddf_predict_optimizer.a 

rddf_metrics: rddf_metrics.o rddf_predict_optimizer.o
rddf_predict_optimizer.a: rddf_predict_optimizer.o rddf_metrics.o

include ../../../Makefile.rules
