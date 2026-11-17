include ../Makefile.conf

# Migração Ubuntu 26.04: o proccontrol_gui era Qt4 + Qt3Support (extinto) e era construído
# à parte, por qmake, a partir do proccontrol.pro. Foi reescrito em GTK3
# (proccontrol_gtk.c), seguindo a mesma solução já usada num fork deste código, e agora
# é um alvo normal deste Makefile — sem qmake, sem moc, sem .pro.
# Ver ../../doc/migracao_ubuntu26/CHANGELOG.md, item 44.
IFLAGS += `pkg-config --cflags gtk+-3.0`
LFLAGS += -lcarmenserial -lglobal -lipc -lespeak -lvoice_interface_interface -lpthread
LFLAGS += -luser_preferences `pkg-config --libs gtk+-3.0`
UBUNTU_VERSION ="$(shell cat /etc/issue | grep Ubuntu | cut -c8-12)"
#$(warning now we reached ifeq UBUNTU_VERSION=$(UBUNTU_VERSION))
ifeq ($(UBUNTU_VERSION),"20.04")
	LFLAGS += -pthread
endif

MODULE_NAME = PROCCONTROL
MODULE_COMMENT = Process controller for CARMEN

SOURCES = 	watchdog.c proccontrol_interface.c proccontrol.c \
			proccontrol_ipc.c proccontrol_setmodule.c \
			proccontrol_setgroup.c proccontrol_viewoutput.c \
			proccontrol_gtk.c

PUBLIC_INCLUDES = 	proccontrol_messages.h proccontrol_interface.h 
PUBLIC_LIBRARIES = 	libproccontrol_interface.a
PUBLIC_BINARIES = 	proccontrol proccontrol_setmodule \
	                proccontrol_setgroup proccontrol_viewoutput proccontrol_gui

TARGETS = 	watchdog libproccontrol_interface.a proccontrol \
			proccontrol_setmodule proccontrol_setgroup \
	 		proccontrol_viewoutput proccontrol_gui

watchdog:			watchdog.o

libproccontrol_interface.a:	proccontrol_interface.o

proccontrol:			proccontrol.o proccontrol_ipc.o

proccontrol_setmodule:		proccontrol_setmodule.o \
				libproccontrol_interface.a

proccontrol_setgroup:		proccontrol_setgroup.o \
				libproccontrol_interface.a

proccontrol_viewoutput:		proccontrol_viewoutput.o \
				libproccontrol_interface.a
proccontrol_gui:		proccontrol_gtk.o \
				libproccontrol_interface.a

include ../Makefile.rules
