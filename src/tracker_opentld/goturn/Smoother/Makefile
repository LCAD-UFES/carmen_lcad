include ../../../Makefile.conf

MODULE_NAME = GOTURN-tracker/Smoother
MODULE_COMMENT = Path smooth and bezier interpolation

LINK = g++

CFLAGS +=
CXXFLAGS += -Wno-reorder
IFLAGS += `pkg-config --cflags opencv` -I Helpers -I Entities

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models
		  
LFLAGS += `pkg-config --libs opencv` -lopencv_video -lglobal \
		-llocalize_ackerman_interface -lobstacle_distance_mapper_interface \
		-lcollision_detection -lsmoother
		
SOURCES = CGSmoother.cpp \
		Entities/Pose2D.cpp \
		Entities/State2D.cpp
				
PUBLIC_INCLUDES = 
    
PUBLIC_LIBRARIES = libsmoother.a
TARGETS = libsmoother.a
PUBLIC_BINARIES = 

libsmoother.a: CGSmoother.o \
			Entities/Pose2D.o \
			Entities/State2D.o

include ../../../Makefile.rules
