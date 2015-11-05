include $(CARMEN_HOME)/src/Makefile.conf

# Module name and description
MODULE_NAME = ILab Saliency Library - Transport
MODULE_COMMENT = Neural Visual Saliency Searcher

LINK = g++

# Application specific include directories.
IFLAGS += -I../ -I/usr/include/libxml2/ -include config.h -include stdlib.h -include string
LFLAGS += -L/usr/lib/x86_64-linux-gnu/ -L/usr/lib/i386-linux-gnu/ -L/usr/local/lib/ -L/usr/lib/libblas/

LFLAGS += -L./Beowulf -L./GUI -L./Neuro -L./Psycho -L./Robots -L./Simulation -L./Transport -L./Video -L./Channels -L./Devices \
-L./HMAX -L./Learn -L./Media -L./nub -L./QtUtil -L./rutz -L./SpaceVariant -L./Util -L./Component -L./Envision -L./Gist \
-L./Image -L./ModelNeuron -L./ObjRec -L./Raster -L./SIFT -L./Surprise -L./VFAT

LFLAGS += -lGL -lXtst -lXext -lX11 -lGLEW -lQtGui -lQtCore -lSDL -lSDL_gfx -lXtst -lXext -lX11 -lbz2 -lcv -lcxcore -lcvaux -lcxcore \
-lcv -lcvaux -ldc1394 -ldl -lglut -lGLU -lGL -lXtst -lXext -lX11 -ljpeg -llapack -lblas -lm -lpng -lpthread -lswscale -lavformat \
-lavcodec -lavutil -lpng -lz -lxml2 -lz

LFLAGS += -lbeowulf -lgui -lneuro -lpsycho -lrobots -lsimulation -lvideo -lchannels -ldevices -lhmax -llearn -lmedia -lnub \
-lqtutil -lrutz -lspacevariant -lutil -lcomponent -lenvision -lgist -limage -lmodelneuron -lobjrec -lraster -lsift -lsurprise -lvfat

# Required default libraries to comunicate with Carmen Core
LFLAGS += 
CFLAGS += -Wno-unused-parameter -Wno-ignored-qualifiers
CXXFLAGS += -Wno-unused-parameter -Wno-ignored-qualifiers

# Source code files (.c, .cpp) 
SOURCES = BarStimuli.C BobDeinterlacer.C BufferedFrameIstream.C CoerceVideoFormatOfilt.C ColorbarsInput.C ColorizeOfilt.C \
Deinterlacer.C DotStimuli.C FrameInfo.C FrameIstream.C FrameIstreamFactory.C FrameOfilt.C FrameOstream.C FrameOstreamFactory.C \
GameOfLifeInput.C HalfFieldDeinterlacer.C HashOutputSeries.C InfoOutputSeries.C LowLevelEncoder.C LowLevelEncoderMap.C \
LuminanceOfilt.C RandomInput.C RasterInputOptions.C RasterInputSeries.C RasterOutputSeries.C RasterlistInputSeries.C \
ShiftedImage.C SplitRgbOfilt.C StatsOutputSeries.C Stimulus2D.C TransportOpts.C World2DInput.C World3DInput.C XMLInput.C

#SUBDIRS = Component Media Transport Video ImageNeuro Raster Simulation Util nub rutz

TARGETS = libtransport.a

libtransport.a: BarStimuli.o BobDeinterlacer.o BufferedFrameIstream.o CoerceVideoFormatOfilt.o ColorbarsInput.o ColorizeOfilt.o \
Deinterlacer.o DotStimuli.o FrameInfo.o FrameIstream.o FrameIstreamFactory.o FrameOfilt.o FrameOstream.o FrameOstreamFactory.o \
GameOfLifeInput.o HalfFieldDeinterlacer.o HashOutputSeries.o InfoOutputSeries.o LowLevelEncoder.o LowLevelEncoderMap.o \
LuminanceOfilt.o RandomInput.o RasterInputOptions.o RasterInputSeries.o RasterOutputSeries.o RasterlistInputSeries.o \
ShiftedImage.o SplitRgbOfilt.o StatsOutputSeries.o Stimulus2D.o TransportOpts.o World2DInput.o World3DInput.o XMLInput.o

# rules

include $(CARMEN_HOME)/src/Makefile.rules
