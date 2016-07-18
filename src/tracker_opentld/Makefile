include ../Makefile.conf

MODULE_NAME = tracker_opentld
MODULE_COMMENT = tracker_opentld

LINK = g++

CXXFLAGS += 
CFLAGS += 
IFLAGS += `pkg-config --cflags opencv` -I 3rdparty/cvblobs -I libopentld/mftracker -I libopentld/tld -I libopentld/imacq -I main_utils -I 3rdparty/libconfig
LFLAGS += `pkg-config --libs opencv` -lopentld  -l3rdparty -lopencv_video -lparam_interface -lipc -lglobal -lbumblebee_basic_interface -lvisual_tracker_interface
SUBDIRS += libopentld 3rdparty

SOURCES = tracker_opentld_main.cpp \
	main_utils/Gui.cpp \
    main_utils/Settings.cpp \
	main_utils/Trajectory.cpp
	
PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES =  
TARGETS = tracker_opentld tracker_test
PUBLIC_BINARIES =

tracker_opentld: tracker_opentld_main.o \
	main_utils/Gui.o \
    main_utils/Settings.o \
	main_utils/Trajectory.o

tracker_test: tracker_opentld_test.o

include ../Makefile.rules
