include $(CARMEN_HOME)/src/Makefile.conf

# Module name and description
MODULE_NAME = WEB_CAM_PACKAGE
MODULE_COMMENT = Web Cam Module

LINK = g++

IFLAGS += 
LFLAGS += -lparam_interface -lipc -lglobal `pkg-config --libs opencv`

SOURCES = web_cam_main.c web_cam_interface.c web_cam_test.c

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = web_cam_interface.h web_cam_messages.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libweb_cam_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = web_cam

TARGETS = libweb_cam_interface.a web_cam web_cam_test

libweb_cam_interface.a: web_cam_interface.o
web_cam: web_cam_interface.o web_cam_main.o
web_cam_test: web_cam_test.o web_cam_interface.o

include $(CARMEN_HOME)/src/Makefile.rules
