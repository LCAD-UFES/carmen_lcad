include ../Makefile.conf

LFLAGS += -lcarmenserial -lglobal -lipc -lespeak -lvoice_interface_interface -lpthread
UBUNTU_VERSION ="$(shell cat /etc/issue | grep Ubuntu | cut -c8-12)"
#$(warning now we reached ifeq UBUNTU_VERSION=$(UBUNTU_VERSION))
ifeq ($(UBUNTU_VERSION),"20.04")
	LFLAGS += -pthread
endif

MODULE_NAME = PROCCONTROL
MODULE_COMMENT = Process controller for CARMEN

SOURCES = 	watchdog.c proccontrol_interface.c proccontrol.c \
			proccontrol_ipc.c proccontrol_setmodule.c \
			proccontrol_setgroup.c proccontrol_viewoutput.c

PUBLIC_INCLUDES = 	proccontrol_messages.h proccontrol_interface.h 
PUBLIC_LIBRARIES = 	libproccontrol_interface.a
PUBLIC_BINARIES = 	proccontrol proccontrol_setmodule \
	                proccontrol_setgroup proccontrol_viewoutput proccontrol_gui

TARGETS = 	watchdog libproccontrol_interface.a proccontrol \
			proccontrol_setmodule proccontrol_setgroup \
	 		proccontrol_viewoutput proccontrol_gui proccontrol_gui 

watchdog:			watchdog.o

libproccontrol_interface.a:	proccontrol_interface.o

proccontrol:			proccontrol.o proccontrol_ipc.o

proccontrol_setmodule:		proccontrol_setmodule.o \
				libproccontrol_interface.a

proccontrol_setgroup:		proccontrol_setgroup.o \
				libproccontrol_interface.a

proccontrol_viewoutput:		proccontrol_viewoutput.o \
				libproccontrol_interface.a
proccontrol_gui: 			../utilities/user_preferences/libuser_preferences.a proccontrol.pro proccontrol_gui.cpp proccontrol_gui.h proccontrol_interface.c
	+make -C ../utilities/user_preferences
	+qmake -o Makefile.qmake-qt5 proccontrol.pro
	+sed -i '/^CXXFLAGS.*/a CXXFLAGS += -Wno-deprecated-copy -Wno-implicit-fallthrough -Wno-deprecated-declarations' Makefile.qmake-qt5
	+make -f Makefile.qmake-qt5

#clean: clean_proccontrol_gui
clean: clean_proccontrol_gui

clean_proccontrol_gui:
	+qmake -o Makefile.qmake-qt5 proccontrol.pro
	+make -f Makefile.qmake-qt5 clean
	+rm -f Makefile.qmake-qt5
	
include ../Makefile.rules
