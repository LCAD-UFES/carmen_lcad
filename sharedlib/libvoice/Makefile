include ../../src/Makefile.conf

LINK=g++
CFLAGS += -std=c99

LFLAGS += -lespeak 
	  
MODULE_NAME = CARMEN VOICE
MODULE_COMMENT = Carmen Voice Synthesizer 

SOURCES = voice.c
#PUBLIC_INCLUDES = voice.h
#PUBLIC_LIBRARIES = libvoice.a

TARGETS = libvoice.a

ifdef CARMEN_DEBUG
CFLAGS += -DDEBUG
endif

libvoice.a: voice.o

include ../../src/Makefile.rules