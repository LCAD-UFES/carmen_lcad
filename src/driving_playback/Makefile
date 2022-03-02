include ../Makefile.conf

# Module name and description
MODULE_NAME = DRIVING PLAYBACK
MODULE_COMMENT = Driving record and playback

LINK = g++

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lipc -pthread -lglobal \
	-lfused_odometry_interface -lrddf_interface -lmotion_planner_interface
		
CFLAGS += -Wall -g

# Source code files (.c, .cpp) 
SOURCES = driving_playback_build.cpp \
	driving_playback_play.cpp driving_playback_index.cpp 

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = 

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES =

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = driving_playback_build driving_playback_play

TARGETS = driving_playback_build driving_playback_play

driving_playback_play: driving_playback_index.o driving_playback_play.o
driving_playback_build: driving_playback_build.o

# rules
include ../Makefile.rules
