
include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = LIB VOICE GOOGLE
MODULE_COMMENT = Library to Synthesize Voice using Google API

SOURCES = main_teste.cpp voice_functions.cpp
IFLAGS += -I/usr/include/python2.7/
LFLAGS += -lpython2.7

TARGETS = main_teste

main_teste: main_teste.o voice_functions.o 

include $(CARMEN_HOME)/src/Makefile.rules

