include ../Makefile.conf

MODULE_NAME = "LOGTOOLS"
MODULE_COMMENT = "Log tools, like the famous log2pic"

#		echo "WAND_CFLAGS     = `$(WAND_CONFIG) --cflags ` -DUSE_WAND" >> $(VARS); \
#		echo "WAND_CXXFLAGS   = `$(WAND_CONFIG) --cppflags `" >> $(VARS); \
#		echo "WAND_LIBS       = `$(WAND_CONFIG) --libs`" >> $(VARS); \


LFLAGS += -lmap_io -lmap_interface -llogtools -lreadlog -lglobal -lipc -lstdc++

ifeq (Fedora release 14 (Laughlin),$(SO_DISTRO))
  LFLAGS += -lstdc++
endif

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src
ifdef QT3
CXXFLAGS += -Wno-non-virtual-dtor
ifeq ($(PROCESSOR),x86_64)
IFLAGS += -I/usr/lib64/qt-3.3/include
LFLAGS += -L/usr/lib64/qt-3.3/lib
else
IFLAGS += -I/usr/lib/qt-3.3/include
LFLAGS += -L/usr/lib/qt-3.3/lib
endif
LFLAGS += -lqt-mt
endif
LFLAGS += -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system

SOURCES = utils.c color.c strings.c mdalloc.c read-script.c read-rec.c \
	  write-file.c parse-lines.c correction.c graphics.c \
	  ll2utm.c utm2ll.c log2log.c clfconvert.c

ifdef QT3
SOURCES += 	display-laserdata.cpp display-laserdata-moc.cpp
endif

PUBLIC_INCLUDES = logtools.h logtools_graphics.h clfconvert.h

PUBLIC_LIBRARIES = liblogtools.a libclfconvert.a

PUBLIC_BINARIES = log2pic log2log

TARGETS = liblogtools.a libclfconvert.a log2log clf-read-example clf-get-oldlaser

ifdef QT3
TARGETS +=		display-laserdata
endif

ifndef NO_WAND
WAND_CFLAGS = `/usr/bin/pkg-config --cflags Wand`
WAND_LDFLAGS = `/usr/bin/pkg-config --libs Wand`
CFLAGS += $(WAND_CFLAGS)
LFLAGS += $(WAND_LDFLAGS) -L/usr/X11R6/lib 
SOURCE += magick-utils.c log2pic.c
PUBLIC_BINARIES += log2pic 
TARGETS += log2pic
endif

liblogtools.a:	utils.o color.o strings.o mdalloc.o read-script.o \
	read-rec.o write-file.o parse-lines.o correction.o \
	graphics.o ll2utm.o utm2ll.o

log2log:	log2log.o

libclfconvert.a: clfconvert.o

clf-read-example: clf-read-example.o

clf-get-oldlaser: clf-get-oldlaser.o

display-laserdata: display-laserdata.o display-laserdata-moc.o

log2pic:	magick-utils.o log2pic.o

# rules

include ../Makefile.rules
