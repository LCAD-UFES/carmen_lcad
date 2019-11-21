include ../../Makefile.conf

CFLAGS += 
LFLAGS += -lglobal -lipc -lparam_interface -llocalize_ackerman_interface -lbumblebee_basic_interface -lgsl -lgslcblas

#G2O
IFLAGS += -I/usr/include/eigen3 
IFLAGS += -I/usr/include/suitesparse -I/usr/include/eigen3 -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/

MODULE_NAME = RDDF_PREDICT
MODULE_COMMENT = 

SOURCES = rddf_predict.cpp rddf_predict_optimizer.cpp
PUBLIC_INCLUDES = rddf_predict_optimizer.h
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = rddf_predict

TARGETS = rddf_predict

rddf_predict: rddf_predict.o rddf_predict_optimizer.o

include ../../Makefile.rules
