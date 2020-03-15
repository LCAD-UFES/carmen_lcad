include ../Makefile.conf

LINK=g++
CFLAGS +=
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcmt/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcmt/ 
LFLAGS += -lipc -lglobal -lparam_interface -lxsens_interface -lrotation_geometry -lcmt -lrt -lpthread

MODULE_NAME = XSENS
MODULE_COMMENT = Xsens Data 

SOURCES = xsens.cpp xsenscore.cpp xsens_interface.c
PUBLIC_INCLUDES = xsens_messages.h xsens_interface.h xsenscore.h

PUBLIC_LIBRARIES = libxsens_interface.a libxsens_core.a
PUBLIC_BINARIES = xsens 
MAN_PAGES =

TARGETS = libcmt.a libxsens_interface.a libxsens_core.a xsens 

PUBLIC_LIBRARIES_SO = libxsens_interface.so

ifndef NO_PYTHON
TARGETS += libxsens_interface.so.1
endif

SOURCES += xsens_listener.c
PUBLIC_BINARIES += xsens_listener
TARGETS += xsens_listener


libcmt.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libcmt

libxsens_interface.a:	xsens_interface.o

libxsens_interface.so.1:	xsens_interface.o

libxsens_core.a: xsenscore.o $(CARMEN_HOME)/sharedlib/libcmt/libcmt.a

xsens:	xsens.o libxsens_core.a libxsens_interface.a

xsens_listener: xsens_listener.o libxsens_interface.a

clean: clean_xsens

clean_xsens:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libcmt

include ../Makefile.rules



