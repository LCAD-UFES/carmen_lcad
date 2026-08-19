include ../Makefile.conf

# Migração Ubuntu 26.04: o módulo linkava contra o OpenCV mas nunca pedia os -I dele.
# No Ubuntu 20.04 os headers ficavam em /usr/include/opencv2 (path padrão); o pacote do
# 26.04 põe tudo em /usr/include/opencv4, que exige -I explícito.
IFLAGS += `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv`


MODULE_NAME = Yolo Save Prediction
MODULE_COMMENT = Salva as detecoes da yolov3 em arquivo, no formato da propria yolo e no formato para calculo de metrica

LINK = g++
CXXFLAGS = -std=c++14

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I$(CARMEN_HOME)/sharedlib/darknet2/src -I$(CARMEN_HOME)/sharedlib/darknet2/include -I/usr/local/include/bullet/ `pkg-config --cflags python3 2>/dev/null || python3-config --includes` -DBOOST_SIGNALS_NO_DEPRECATION_WARNING 

LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet2/lib -L$(CARMEN_HOME)/sharedlib/libtf/src

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libsickldmrs2/include
# Required default libraries to comunicate with Carmen Core.
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libsickldmrs2 

LFLAGS += -lglobal -lipc -ldarknet2 `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv` -ltf -lpthread \
			-lLinearMath -lrt `pkg-config --libs python3-embed 2>/dev/null || python3-config --embed --libs`

SOURCES = yolo_save_prediction.cpp 

PUBLIC_BINARIES = yolo_save_prediction

TARGETS = darknet2 yolo_save_prediction

yolo_save_prediction: yolo_save_prediction.o

darknet2:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/darknet2

include ../Makefile.rules
