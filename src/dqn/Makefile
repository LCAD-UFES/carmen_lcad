
include ../Makefile.conf

# Module name and description
MODULE_NAME = DQN
MODULE_COMMENT = Deep Q-Network

LINK = g++

# Required default libraries to comunicate with Carmen Core
IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -I../car_panel_gui/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -L../car_panel_gui/

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lparam_interface -lipc -lglobal `pkg-config --libs opencv` -lgrid_mapping_interface -lrddf_interface \
	-llocalize_ackerman_interface -lrobot_ackerman_interface -lrddf_index -lprob_models -lmap_server_interface \
	-lcollision_detection -lmap_interface -lrotation_geometry -lgrid_mapping -lmap_io
CFLAGS += -Wall -Wno-write-strings 
CXXFLAGS += -Wall -Wno-write-strings

# Source code files (.c, .cpp) 
SOURCES = dqn_emulator.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = 

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = 

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = dqn_emulator 

TARGETS = dqn_emulator

dqn_emulator: dqn_emulator.o

# rules

include ../Makefile.rules

