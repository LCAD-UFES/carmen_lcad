include ../../src/Makefile.conf

LINK=g++
CFLAGS += -std=c99 -Wno-write-strings
CXXFLAGS += -std=c++0x -Wno-write-strings

LFLAGS += -lespeak 
	  
MODULE_NAME = CARMEN VOICE
MODULE_COMMENT = Carmen Voice Synthesizer 

SOURCES = voice.cpp voice_test.cpp
#PUBLIC_INCLUDES = voice.h
#PUBLIC_LIBRARIES = libvoice.a

TARGETS = libvoice.a voice_test

ifdef CARMEN_DEBUG
CFLAGS += -DDEBUG
endif

libvoice.a: voice.o
voice_test: voice_test.o voice.o

include ../../src/Makefile.rules
