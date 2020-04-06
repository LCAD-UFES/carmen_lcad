include ../../Makefile.conf

MODULE_NAME = User Preferences
MODULE_COMMENT = Read and save user preference parameters 

LINK = g++
CFLAGS += -Wall
CXXFLAGS += -Wno-ignored-qualifiers -std=c++11

IFLAGS += 

LFLAGS += \
	-lparam_interface

SOURCES = \
	user_preferences.cpp

PUBLIC_INCLUDES = \
	user_preferences.h

PUBLIC_LIBRARIES = \
	libuser_preferences.a

PUBLIC_BINARIES = 

TARGETS = \
	libuser_preferences.a

libuser_preferences.a: user_preferences.o

include ../../Makefile.rules
