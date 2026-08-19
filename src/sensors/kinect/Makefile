include $(CARMEN_HOME)/src/Makefile.conf

LINK=g++

# ---------------------------------------------------------------------------------------
# Migração Ubuntu 26.04: este módulo depende da libfreenect (driver do Kinect). Ela existe
# no apt (libfreenect-dev), mas não vem instalada por padrão e nem todo carro usa Kinect —
# sem ela o módulo derrubava o build inteiro. Agora é detectada: sem a lib, sai só a
# libkinect_interface (wrapper IPC); instalando `libfreenect-dev`, o módulo volta a ser
# construído inteiro, sem editar nada.
# ---------------------------------------------------------------------------------------
HAS_FREENECT := $(firstword $(wildcard /usr/include/libfreenect/libfreenect.h /usr/include/libfreenect.h /usr/local/include/libfreenect/libfreenect.h))

CFLAGS += -std=c99
IFLAGS += -I/usr/local/include/libfreenect -I/usr/include/libusb-1.0
LFLAGS += -lglobal -lparam_interface -lipc -lfreenect -llaser_interface -lGL -lGLU -lglut

MODULE_NAME = KINECT
MODULE_COMMENT = Kinect Data 

ifneq ($(HAS_FREENECT),)
SOURCES = kinect.c kinect_mrds.c kinect_laser.c kinect_interface.c kinect_util.c
else
SOURCES = kinect_interface.c kinect_util.c
endif
PUBLIC_INCLUDES = kinect_messages.h kinect_interface.h kinect_util.h

PUBLIC_LIBRARIES = libkinect_interface.a
MAN_PAGES =

TARGETS = libkinect_interface.a
ifneq ($(HAS_FREENECT),)
PUBLIC_BINARIES = kinect kinect_mrds kinect_laser
TARGETS += kinect kinect_mrds kinect_laser kinect_wrapper.a
endif

PUBLIC_LIBRARIES_SO = 

ifndef NO_PYTHON
TARGETS += 
endif

#NO_GRAPHICS=1
ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += -lparam_interface -lipc -lglobal -lglobal_graphics \
	`$(GTK_CONFIG) --libs` 
ifneq ($(HAS_FREENECT),)
SOURCES += kinect_view.c
PUBLIC_BINARIES += kinect_view
TARGETS += kinect_view
endif
endif

ifdef CARMEN_DEBUG
CFLAGS += -DDEBUG
endif

kinect_wrapper.a:
	$(MAKE) -C kinect_wrapper

libkinect_interface.a: kinect_interface.o kinect_util.o

kinect: kinect.o kinect_wrapper.a libkinect_interface.a

kinect_mrds: kinect_mrds.o libkinect_interface.a

kinect_view: kinect_view.o libkinect_interface.a

kinect_laser: kinect_laser.o libkinect_interface.a

include $(CARMEN_HOME)/src/Makefile.rules

clean: clean_kinect_wrapper

clean_kinect_wrapper:
	$(MAKE) clean -C kinect_wrapper
