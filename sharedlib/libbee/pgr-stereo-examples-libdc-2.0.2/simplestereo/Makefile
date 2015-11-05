
#####################################################################
# Copyright (c) 2008 Point Grey Research Inc.
#
# This Makefile is free software; Point Grey Research Inc.
# gives unlimited permission to copy and/or distribute it,
# with or without modifications, as long as this notice is preserved.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY, to the extent permitted by law; without
# even the implied warranty of MERCHANTABILITY or FITNESS FOR A
# PARTICULAR PURPOSE.
#
#####################################################################

ROOT_INCLUDE = /usr/local/include
ROOT_LIB = /usr/local/lib
LOCALLIB = ../pgrlibdcstereo

TRICLOPS_INCLUDE = ../../triclops_64/include
TRICLOPS_LIB = ../../triclops_64/lib

# compilation flags
CPPFLAGS += -I.

# libdc1394 installed in /usr/local/include location
CPPFLAGS += -I$(ROOT_INCLUDE)/dc1394
CPPFLAGS += -I$(LOCALLIB)
CPPFLAGS += -I$(TRICLOPS_INCLUDE)/
CPPFLAGS += -Wall -g
CPPFLAGS += -DLINUX
#CPPFLAGS += -Wall -O3

LDFLAGS	+= -L. -L$(TRICLOPS_LIB)/
LDFLAGS += -L$(LOCALLIB)
LDFLAGS += -L$(ROOT_LIB)
LIBS    += -ldc1394 -lraw1394 -pthread
LIBS	+= -lpgrlibdcstereo -ltriclops -lpnmutils

# executable name and contents
EXEC1		= simplestereo
EXEC1SRC	= $(EXEC1).cpp

EXEC2		= simplestereo-profile
EXEC2SRC	= $(EXEC2).cpp

EXECS		= $(EXEC1) $(EXEC2)


all:	bin

bin: $(EXECS)

$(EXEC1): $(EXEC1SRC:%.cpp=%.o)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LIBS)

$(EXEC2): $(EXEC2SRC:%.cpp=%.o)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LIBS)

%.o:%.cpp
	g++ -c $(CXXFLAGS) $(CPPFLAGS) $*.cpp -o $*.o

clean:
	rm -f *~ *.o *.d $(EXECS) *.pgm *.ppm

#####################################################################
#
# $Id: Makefile,v 1.2 2008/12/11 00:00:21 warrenm Exp $
# $Author: warrenm $
# $Revision: 1.2 $
# $Date: 2008/12/11 00:00:21 $
#
#####################################################################

