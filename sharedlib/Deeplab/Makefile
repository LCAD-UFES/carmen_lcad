include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CFLAGS += 
CXXFLAGS += -std=c++11

MODULE_NAME = Deeplab Neural Network
MODULE_COMMENT = Semantic segmentation inference using Deeplab tensorflow model

IFLAGS += -I/usr/include/python2.7
LFLAGS += -ldeeplab -lpython2.7 -lpthread `pkg-config --libs opencv` 

SOURCES = generate_semantic_map_images.cpp
 
PUBLIC_INCLUDES = libdeeplab.h
PUBLIC_LIBRARIES = libdeeplab.a
#PUBLIC_BINARIES =

TARGETS = libdeeplab.a generate_semantic_map_images
PUBLIC_LIBRARIES_SO =

libdeeplab.a: libdeeplab.o

install_python2:
	sudo apt-get install python-dev python-pip

generate_semantic_map_images: generate_semantic_map_images.o

# https://stackoverflow.com/questions/7369145/activating-a-virtualenv-using-a-shell-script-doesnt-seem-to-work
virtualenv:
	. create_env.sh

download:
	# wget -O deeplab_model_cityscapes.tar.gz http://download.tensorflow.org/models/deeplabv3_cityscapes_train_2018_02_06.tar.gz
	# Google Drive  user: suporte@lcad.inf.ufes.br  folder: Deeplab  file: deeplab_model_cityscapes.tar.gz  
	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1u4oeKh6I_LezzEGR55Ed4JZ-kDCF5HrH/edit deeplab_model_cityscapes.tar.gz
	

download_lite:
	# wget -O deeplab_model_cityscapes_lite.tar.gz http://download.tensorflow.org/models/deeplabv3_mnv2_cityscapes_train_2018_02_05.tar.gz
	# Google Drive  user: suporte@lcad.inf.ufes.br  folder: Deeplab  file: deeplab_model_cityscapes_lite.tar.gz  
	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1S47c_upLQ7om2An7Ux8UUAtanX1oE8W9/edit deeplab_model_cityscapes_lite.tar.gz

include $(CARMEN_HOME)/src/Makefile.rules