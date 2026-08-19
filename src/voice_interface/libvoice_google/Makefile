
include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = LIB VOICE GOOGLE
MODULE_COMMENT = Library to Synthesize Voice using Google API

SOURCES = main_teste.cpp voice_functions.cpp
IFLAGS += `pkg-config --cflags python3 2>/dev/null || python3-config --includes`
LFLAGS += `pkg-config --libs python3-embed 2>/dev/null || python3-config --embed --libs`

TARGETS = main_teste

main_teste: main_teste.o voice_functions.o 

include $(CARMEN_HOME)/src/Makefile.rules

