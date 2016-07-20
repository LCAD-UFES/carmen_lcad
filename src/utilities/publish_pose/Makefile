include ../../Makefile.conf

CFLAGS += 
IFLAGS += 
LFLAGS += -lipc -lglobal -llocalize_ackerman_interface

MODULE_NAME = PUBLISH_POSE
MODULE_COMMENT = Publish a fixed globalpose

SOURCES = util_publish_pose.cpp
PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES =  
PUBLIC_BINARIES = util_publish_pose

TARGETS = util_publish_pose

util_publish_pose: util_publish_pose.o

include ../../Makefile.rules
