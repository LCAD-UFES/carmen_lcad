include $(CARMEN_HOME)/src/Makefile.conf

# Module name and description
MODULE_NAME = ILab Saliency Library - Media
MODULE_COMMENT = Neural Visual Saliency Searcher

LINK = g++

# Application specific include directories.
IFLAGS += -I../ -I/usr/include/libxml2/ -I/usr/include/qt4/ -include config.h
LFLAGS += -L/usr/lib/x86_64-linux-gnu/ -L/usr/lib/i386-linux-gnu/ -L/usr/local/lib/ -L/usr/lib/libblas/

LFLAGS += -L./Beowulf -L./GUI -L./Neuro -L./Psycho -L./Robots -L./Simulation -L./Transport -L./Video -L./Channels -L./Devices \
-L./HMAX -L./Learn -L./Media -L./nub -L./QtUtil -L./rutz -L./SpaceVariant -L./Util -L./Component -L./Envision -L./Gist \
-L./Image -L./ModelNeuron -L./ObjRec -L./Raster -L./SIFT -L./Surprise -L./VFAT

LFLAGS += -lGL -lXtst -lXext -lX11 -lGLEW -lQtGui -lQtCore -lSDL -lSDL_gfx -lXtst -lXext -lX11 -lbz2 -lcv -lcxcore -lcvaux -lcxcore \
-lcv -lcvaux -ldc1394 -ldl -lglut -lGLU -lGL -lXtst -lXext -lX11 -ljpeg -llapack -lblas -lm -lpng -lpthread -lswscale -lavformat \
-lavcodec -lavutil -lpng -lz -lxml2 -lz -lipc -lglobal -lbumblebee_basic_interface

LFLAGS += -lbeowulf -lgui -lneuro -lpsycho -lrobots -lsimulation -ltransport -lvideo -lchannels -ldevices -lhmax -llearn -lnub \
-lqtutil -lrutz -lspacevariant -lutil -lcomponent -lenvision -lgist -limage -lmodelneuron -lobjrec -lraster -lsift -lsurprise -lvfat

CFLAGS += -std=c99 -Wno-unused-parameter -Wno-ignored-qualifiers
CXXFLAGS += -Wno-unused-parameter -Wno-ignored-qualifiers

# Source code files (.c, .cpp) 
SOURCES = FfmpegDecoder.C FfmpegEncoder.C FfmpegPacketDecoder.C FrameCounter.C FrameRange.C FrameSeries.C IpcFrameSeries.C HttpEncoder.C \
HttpOutputStream.C MediaOpts.C MediaSimEvents.C MgzDecoder.C MgzEncoder.C MgzInputStream.C MgzJDecoder.C MgzJEncoder.C \
MgzJInputStream.C MgzJOutputStream.C MgzOutputStream.C MovieDecoder.C MpegInputStream.C MpegOutputStream.C MrawvDecoder.C \
MrawvEncoder.C MrawvInputStream.C MrawvOutputStream.C NullOutputStream.C SequenceFileStream.C SimFrameSeries.C TestImages.C \
UcbMpegEncoder.C UcbMpegOutputStream.C

#SUBDIRS = Component Media Transport Video ImageNeuro Raster Simulation Util nub rutz

TARGETS = libmedia.a

libmedia.a: FfmpegDecoder.o FfmpegEncoder.o FfmpegPacketDecoder.o FrameCounter.o FrameRange.o FrameSeries.o IpcFrameSeries.o HttpEncoder.o \
HttpOutputStream.o MediaOpts.o MediaSimEvents.o MgzDecoder.o MgzEncoder.o MgzInputStream.o MgzJDecoder.o MgzJEncoder.o \
MgzJInputStream.o MgzJOutputStream.o MgzOutputStream.o MovieDecoder.o MpegInputStream.o MpegOutputStream.o MrawvDecoder.o \
MrawvEncoder.o MrawvInputStream.o MrawvOutputStream.o NullOutputStream.o SequenceFileStream.o SimFrameSeries.o TestImages.o \
UcbMpegEncoder.o UcbMpegOutputStream.o

# rules

include $(CARMEN_HOME)/src/Makefile.rules
