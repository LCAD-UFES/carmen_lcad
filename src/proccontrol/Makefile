include ../Makefile.conf

LFLAGS += -lcarmenserial -lglobal -lipc -lespeak -lvoice_interface_interface

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
proccontrol_gui: 			proccontrol_gui.cpp proccontrol_gui.h proccontrol_interface.c
	+qmake-qt4 -o Makefile.qmake-qt4
	+make -f Makefile.qmake-qt4
	
#clean: clean_proccontrol_gui
clean: clean_proccontrol_gui

clean_proccontrol_gui:
	+qmake-qt4 -o Makefile.qmake-qt4
	+make -f Makefile.qmake-qt4 clean
	+rm Makefile.qmake-qt4
	
include ../Makefile.rules
