#############################################################################
#
#	Makefile for building the serial sample program
#
#############################################################################


TARGET	= test-Serial

PWD     := $(shell pwd)

CC = /usr/bin/gcc
CXX = /usr/bin/g++

# If you need additional serch paths for include files, then use the -I flag
# and add them to the CPPFLAGS variable
#

CPPFLAGS += -I.

#
# If you need addional search paths for library files, then use the -L flag
# and add them to LDFLAGS.
#

LDFLAGS = -c

#
# If you need additional libraries, then use -lxxx to search for libxxx.a
#

# LDLIBS += -lxxx

Serial.o = $(CXX) $(LDFLAGS) Serial.C 

all:
	g++ Serial.C test-Serial.C -o test-Serial


#
# You can change the $(TARGET).c if that's not your main file and you can
# add additional .o files to the end of the line
#

SRCS	= $(TARGET).C Serial.C
OBJS	= Serial.o $(TARGET).o

$(TARGET) : $(OBJS)

clean:
	rm -rf $(OBJS) core .depend $(TARGET)


