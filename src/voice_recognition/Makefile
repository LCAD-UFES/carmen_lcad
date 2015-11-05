include ../Makefile.conf

# Module name and description
MODULE_NAME = VOICE RECOGNITION
MODULE_COMMENT = Neural Voice Recognition Module 

LINK = g++
CFLAGS += -Wno-unused-parameter

# Application specific include directories.
IFLAGS += -I$(MAEHOME)/include
IFLAGS += `pkg-config --cflags opencv`

# Required default libraries to comunicate with Carmen Core.
LFLAGS += `pkg-config --libs opencv`
LFLAGS += -L$(MAEHOME)/lib -L/usr/local/lib -L/usr/X11R6/lib
LFLAGS += -lnet_conn -lforms -lGA -lX11 -lglut -lGLU -lGL -lm -lfl -lrt -lz -fopenmp 	\
		  -lparam_interface -lipc -lglobal -lmap_interface -lasound -lfftw3

MAE_FILES = voice_recognition.con
# Source code files (.c, .cpp) 
SOURCES = voice_recognition.c voice_recognition_user_functions/voice_recognition_user_functions.c \
		  voice_recognition_user_functions/voice_recognition_filters.c voice_recognition_user_functions/voice_recognition_utils.c \
		  voice_recognition_interface.c voice_recognition_user_functions/ring_buffer.c voice_recognition_test.cpp

PUBLIC_INCLUDES = voice_recognition_interface.h  voice_recognition_messages.h
# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libvoice_recognition_interface.a 

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = voice_recognition voice_recognition_test

TARGETS = voice_recognition libvoice_recognition_interface.a voice_recognition_test

libvoice_recognition_interface.a: voice_recognition_interface.o voice_recognition_user_functions/ring_buffer.o
voice_recognition: voice_recognition.o voice_recognition_user_functions/voice_recognition_user_functions.o voice_recognition_user_functions/voice_recognition_filters.o voice_recognition_user_functions/voice_recognition_utils.o voice_recognition_user_functions/ring_buffer.o libvoice_recognition_interface.a 
voice_recognition_test: voice_recognition_test.o libvoice_recognition_interface.a

clean_user_functions:
	rm -rf *.con~
	rm -rf *.core
	rm -rf *.cml~
	$(MAKE) clean -C voice_recognition_user_functions

clean:	clean_user_functions

# rules

include ../Makefile.rules

