include ../../Makefile.conf

CFLAGS += 
IFLAGS += 
LFLAGS += -lipc -pthread -lglobal -llocalize_ackerman_interface -lrddf_interface -lparam_interface -lplayback_interface -lgps_nmea_interface

MODULE_NAME = PUBLISH_POSE
MODULE_COMMENT = Publish a fixed globalpose

SOURCES = util_publish_pose.cpp util_publish_initial_pose.cpp util_publish_final_goal.cpp util_publish_dummy_gps.cpp util_publish_gps_from_globalpos.cpp
PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES =  
PUBLIC_BINARIES = util_publish_pose util_publish_initial_pose util_publish_final_goal util_publish_dummy_gps util_publish_gps_from_globalpos

TARGETS = util_publish_pose util_publish_initial_pose util_publish_final_goal util_publish_dummy_gps util_publish_gps_from_globalpos

util_publish_pose: util_publish_pose.o 

util_publish_initial_pose: util_publish_initial_pose.o

util_publish_final_goal: util_publish_final_goal.o

util_publish_dummy_gps: util_publish_dummy_gps.o

util_publish_gps_from_globalpos: util_publish_gps_from_globalpos.o

include ../../Makefile.rules
