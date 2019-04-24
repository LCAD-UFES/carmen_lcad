include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = "CVCAM"
MODULE_COMMENT = "Publishes images from camera"

SOURCES = cvcam.c
TARGETS = libcvcam.a 

IFLAGS += `pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv`

all: 

libcvcam.a: cvcam.o

# rules

include $(CARMEN_HOME)/src/Makefile.rules
