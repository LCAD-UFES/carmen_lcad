
include $(CARMEN_HOME)/src/Makefile.conf

LINK=g++
CFLAGS += -std=c99 -Wno-write-strings  
CXXFLAGS += -std=c++0x -Wno-write-strings 

LFLAGS += -lespeak 
	  
MODULE_NAME = LIB VOICE
MODULE_COMMENT = Library to Synthesize Voice

SOURCES = voice.cpp voice_test.cpp
PUBLIC_INCLUDES = voice.h
PUBLIC_LIBRARIES = libvoice.a

TARGETS = libvoice.a voice_test

ifdef CARMEN_DEBUG
CFLAGS += -DDEBUG
endif

libvoice.a: voice.o
voice_test: voice_test.o voice.o

include $(CARMEN_HOME)/src/Makefile.rules
