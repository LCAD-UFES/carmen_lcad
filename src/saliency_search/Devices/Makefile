include $(CARMEN_HOME)/src/Makefile.conf

# Module name and description
MODULE_NAME = ILab Saliency Library - Devices
MODULE_COMMENT = Neural Visual Saliency Searcher

LINK = g++

# Application specific include directories.
IFLAGS += -I../ -I/usr/include/libxml2/ -include config.h
LFLAGS += -L/usr/lib/x86_64-linux-gnu/ -L/usr/lib/i386-linux-gnu/ -L/usr/local/lib/ -L/usr/lib/libblas/

LFLAGS += -L./Beowulf -L./GUI -L./Neuro -L./Psycho -L./Robots -L./Simulation -L./Transport -L./Video -L./Channels -L./Devices \
-L./HMAX -L./Learn -L./Media -L./nub -L./QtUtil -L./rutz -L./SpaceVariant -L./Util -L./Component -L./Envision -L./Gist \
-L./Image -L./ModelNeuron -L./ObjRec -L./Raster -L./SIFT -L./Surprise -L./VFAT

LFLAGS += -lGL -lXtst -lXext -lX11 -lGLEW -lQtGui -lQtCore -lSDL -lSDL_gfx -lXtst -lXext -lX11 -lbz2 -lcv -lcxcore -lcvaux -lcxcore \
-lcv -lcvaux -ldc1394 -ldl -lglut -lGLU -lGL -lXtst -lXext -lX11 -ljpeg -llapack -lblas -lm -lpng -lpthread -lswscale -lavformat \
-lavcodec -lavutil -lpng -lz -lxml2 -lz

LFLAGS += -lbeowulf -lgui -lneuro -lpsycho -lrobots -lsimulation -ltransport -lvideo -lchannels -lhmax -llearn -lmedia -lnub \
-lqtutil -lrutz -lspacevariant -lutil -lcomponent -lenvision -lgist -limage -lmodelneuron -lobjrec -lraster -lsift -lsurprise -lvfat
# Required default libraries to comunicate with Carmen Core
LFLAGS += 
CFLAGS += -std=c99 -Wall
CXXFLAGS += -std=c++0x -Wno-unused-parameter -Wno-ignored-qualifiers

# Source code files (.c, .cpp) 
SOURCES =  Bumblebee2Grabber.C DC1394Grabber2.C DeviceOpts.C DiskDataStream.C HTTPServer.C IEEE1394grabber.C KinectGrabber.C \
OpenNIGrabber.C QuickTimeGrabber.C RTSPGrabber.C V4L2grabber.C V4Lgrabber.C XCgrabber.C XCgrabberFlex.C

#SUBDIRS = Component Media Transport Video ImageNeuro Raster Simulation Util nub rutz

TARGETS = libdevices.a

libdevices.a: Bumblebee2Grabber.o DC1394Grabber2.o DeviceOpts.o DiskDataStream.o HTTPServer.o IEEE1394grabber.o KinectGrabber.o \
OpenNIGrabber.o QuickTimeGrabber.o RTSPGrabber.o V4L2grabber.o V4Lgrabber.o XCgrabber.o XCgrabberFlex.o

# rules

include $(CARMEN_HOME)/src/Makefile.rules
