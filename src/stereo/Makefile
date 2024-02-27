include ../Makefile.conf

MODULE_NAME = SIMPLE_STEREO
MODULE_COMMENT = Only Stereo Module

CC = g++

ifndef NO_CUDA

CUDALIB64 :=/usr/local/cuda/lib64/
CUDAINCLUDE :=/usr/local/cuda/include/

CFLAGS += 
IFLAGS += -I$(CUDAINCLUDE)
LFLAGS += -L$(CUDALIB64) -lcuda -lcudart -lcublas

endif

LINK = gcc

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libstereovgram
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libstereo
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libelas

IFLAGS += `pkg-config --cflags opencv4`
LFLAGS += `pkg-config --libs opencv4`

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libstereo
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libstereovgram
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libelas
LFLAGS += -lparam_interface -lipc -lglobal -lbumblebee_basic_interface -lelas -lstereovgram -lstereo -lstdc++ -fopenmp -lm

CFLAGS += -O3 -fopenmp
CXXFLAGS += -O3 -fopenmp

SOURCES = stereo_main.cpp  stereo_interface.c \
stereo_util.c test_bias_reduction.cpp test_bias_reduction_with_check_board.cpp stereo_test.cpp
PUBLIC_INCLUDES = stereo_interface.h stereo_messages.h stereo_util.h
PUBLIC_LIBRARIES = libstereo_interface.a 
PUBLIC_BINARIES = simple_stereo
TARGETS = libelas.a libstereovgram.a libstereo.a libstereo_interface.a simple_stereo stereo_test 
PUBLIC_LIBRARIES_SO = libbstereo_interface.so
ifndef NO_PYTHON
TARGETS += libstereo_interface.so.1
endif

ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += -lparam_interface -lipc -lglobal -lglobal_graphics -lstereo_interface \
	`$(GTK_CONFIG) --libs` 
SOURCES += stereo_view.c
PUBLIC_BINARIES += stereo_view test_bias_reduction
TARGETS += stereo_view test_bias_reduction test_bias_reduction_with_check_board 
endif

libelas.a:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libelas

libstereo.a:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libstereo

libstereovgram.a:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libstereovgram

libstereo_interface.a:		stereo_interface.o stereo_util.o

libstereo_interface.so.1:	stereo_interface.o

simple_stereo:			stereo_main.o libstereo_interface.a

stereo_test: 			stereo_test.o

stereo_view:			stereo_view.o libstereo_interface.a

test_bias_reduction:		test_bias_reduction.o stereo_util.o
test_bias_reduction_with_check_board:		test_bias_reduction_with_check_board.o stereo_util.o

clean: clean_stereo

clean_stereo:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libstereo
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libstereovgram
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libelas

# rules

include ../Makefile.rules
