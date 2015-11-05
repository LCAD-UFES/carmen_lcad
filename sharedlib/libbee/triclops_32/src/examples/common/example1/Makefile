#####################################################################
# Copyright (c) 2001 Point Grey Research, Inc. All Rights Reserved.
#
# This software is the confidential and proprietary information of Point
# Grey Research, Inc. ("Confidential Information").  You shall not
# disclose such Confidential Information and shall use it only in
# accordance with the terms of the license agreement you entered into
# with PGR.
#
# PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
# SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
# SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
# THIS SOFTWARE OR ITS DERIVATIVES.
#####################################################################
#
# $Id: Makefile,v 1.4 2010/07/12 22:11:28 arturp Exp $
# $Author: arturp $
# $Revision: 1.4 $
# $Date: 2010/07/12 22:11:28 $
#
#####################################################################

TOP	= ../../../..

# compilation flags
CPPFLAGS += -I$(TOP)/include
CPPFLAGS += -I.
CPPFLAGS += -Wall -g
#CPPFLAGS += -Wall -O3

#
# libraries needed by the test drivers if any
#
LDFLAGS += -L$(TOP)/lib 
LIBS    += -ltriclops -lpnmutils -pthread -ldl -lm

EXEC1NAME	= example1
EXEC1SRC 	= $(EXEC1NAME).cpp
EXEC1          = $(EXEC1NAME)
EXECS           = $(EXEC1)

# Default build : test driver in local directory
all:	bin

bin: $(EXECS)

$(EXEC1): $(EXEC1SRC:%.cpp=%.o)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LIBS)

%.o:%.cpp
	g++ -c $(CXXFLAGS) $(CPPFLAGS) $*.cpp

clean:
	rm -f *~ *.o *.d bin
