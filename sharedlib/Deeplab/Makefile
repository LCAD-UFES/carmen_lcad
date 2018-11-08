include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CFLAGS += 
CXXFLAGS += 

MODULE_NAME = Deeplab Neural Network
MODULE_COMMENT = Semantic segmentation inference using Deeplab tensorflow model

IFLAGS += -I/usr/include/python2.7
LFLAGS += -lpython2.7 -lpthread

SOURCES = 
 
PUBLIC_INCLUDES = libdeeplab.h
PUBLIC_LIBRARIES = libdeeplab.a
#PUBLIC_BINARIES =

TARGETS = libdeeplab.a  
PUBLIC_LIBRARIES_SO =

libdeeplab.a: libdeeplab.o

install_python2:
	sudo apt-get install python-dev python-pip

# https://stackoverflow.com/questions/7369145/activating-a-virtualenv-using-a-shell-script-doesnt-seem-to-work
virtualenv:
	source create_env.sh

download:
	wget -O deeplab_model_cityscapes.tar.gz http://download.tensorflow.org/models/deeplabv3_cityscapes_train_2018_02_06.tar.gz

download_lite:
	wget -O deeplab_model_cityscapes_lite.tar.gz http://download.tensorflow.org/models/deeplabv3_mnv2_cityscapes_train_2018_02_05.tar.gz

include $(CARMEN_HOME)/src/Makefile.rules