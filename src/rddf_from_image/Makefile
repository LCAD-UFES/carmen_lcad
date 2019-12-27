include ../Makefile.conf

LINK = g++
CFLAGS += 
LFLAGS += -lglobal -lrddf_interface -lipc -lparam_interface -llocalize_ackerman_interface -lbumblebee_basic_interface -lgsl -lgslcblas

#G2O
IFLAGS += -I/usr/include/eigen3 
IFLAGS += -I/usr/include/suitesparse -I/usr/include/eigen3 -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/

#inplace_abn
IFLAGS += -I/usr/include/python3.6m
LFLAGS += -linplace_abn `python3.6-config --libs` 

MODULE_NAME = RDDF_PREDICT
MODULE_COMMENT = 

SOURCES = rddf_from_image.cpp rddf_predict_optimizer.cpp
PUBLIC_INCLUDES = rddf_predict_optimizer.h
PUBLIC_LIBRARIES = rddf_predict_optimizer.a
PUBLIC_BINARIES = rddf_from_image

TARGETS = rddf_from_image rddf_predict_optimizer.a 

rddf_from_image: rddf_from_image.o rddf_predict_optimizer.o
rddf_predict_optimizer.a: rddf_predict_optimizer.o rddf_from_image.o

libinplace_abn.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/inplace_abn/scripts/

include ../Makefile.rules
