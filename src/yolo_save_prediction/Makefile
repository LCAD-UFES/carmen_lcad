include ../Makefile.conf

MODULE_NAME = Yolo Save Prediction
MODULE_COMMENT = Salva as detecoes da yolov3 em arquivo, no formato da propria yolo e no formato para calculo de metrica

LINK = g++
CXXFLAGS = -std=c++11

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I$(CARMEN_HOME)/sharedlib/darknet2/src -I$(CARMEN_HOME)/sharedlib/darknet2/include -I/usr/local/include/bullet/ -I/usr/include/python2.7 -DBOOST_SIGNALS_NO_DEPRECATION_WARNING 

LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet2/lib -L$(CARMEN_HOME)/sharedlib/libtf/src

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libsickldmrs2/include
# Required default libraries to comunicate with Carmen Core.
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libsickldmrs2 

LFLAGS += -lglobal -lipc -ldarknet2 `pkg-config --libs opencv` -ltf -lpthread \
			-lLinearMath -lrt `python2.7-config --libs`

SOURCES = yolo_save_prediction.cpp 

PUBLIC_BINARIES = yolo_save_prediction

TARGETS = darknet2 yolo_save_prediction

yolo_save_prediction: yolo_save_prediction.o

darknet2:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/darknet2

include ../Makefile.rules
