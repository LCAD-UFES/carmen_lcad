include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = TRAFFIC_LIGHT RECOG 
MODULE_COMMENT = 

CFLAGS += -std=c++0x
CXXFLAGS += -std=c++0x

SOURCES = tlight_vgram.cpp
PUBLIC_INCLUDES = tlight_state_recog.h tlight_factory.h tlight_mlp.h tlight_vgram.h

PUBLIC_LIBRARIES = lib_tlight_state_recog.a
PUBLIC_BINARIES = 
TARGETS = lib_tlight_state_recog.a

lib_tlight_state_recog.a: tlight_vgram.o 

# rules
include $(CARMEN_HOME)/src//Makefile.rules
