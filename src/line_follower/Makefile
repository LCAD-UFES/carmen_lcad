include $(CARMEN_HOME)/src/Makefile.conf

CFLAGS += 
IFLAGS += 
LFLAGS += -lipc -lglobal -llocalize_ackerman_interface -lrobot_ackerman_interface -lbase_ackerman_interface \
	-lrddf_interface -lbehavior_selector_interface

IFLAGS += -I/usr/include/eigen3 
IFLAGS += -I/usr/include/suitesparse -I/usr/include/eigen3 -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/

MODULE_NAME = LINE_FOLLOWER
MODULE_COMMENT = A simple navigator to follow a line

SOURCES = line_follower_main.cpp
PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES =  
PUBLIC_BINARIES = line_follower_main

TARGETS = line_follower_main

line_follower_main: line_follower_main.o

include $(CARMEN_HOME)/src/Makefile.rules
