include ../Makefile.conf

MODULE_NAME = SOUND APPLICATIONS  
MODULE_COMMENT = Module for Sound Capture and Reproduction

LINK = g++

IFLAGS += 
LFLAGS += -lparam_interface -lipc -lglobal -lsound_interface -lm -lasound -lfftw3 `pkg-config --libs opencv`
CFLAGS += -Wall

SOURCES = sound_interface.c sound_capture_main.c sound_reproduction_main.c sound.c
PUBLIC_INCLUDES = sound_interface.h sound_messages.h
PUBLIC_LIBRARIES = libsound_interface.a
PUBLIC_BINARIES = sound_capture sound_reproduction
TARGETS = libsound_interface.a sound_capture sound_reproduction

libsound_interface.a: sound_interface.o
sound_capture: sound_interface.o sound_capture_main.o sound.o
sound_reproduction: sound_reproduction_main.o sound_interface.o sound.o
 
# rules
include ../Makefile.rules
