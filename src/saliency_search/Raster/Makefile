include $(CARMEN_HOME)/src/Makefile.conf

# Module name and description
MODULE_NAME = ILab Saliency Library - Raster
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

LFLAGS += -lbeowulf -lgui -lneuro -lpsycho -lrobots -lsimulation -ltransport -lvideo -lchannels -ldevices -lhmax -llearn -lmedia -lnub \
-lqtutil -lrutz -lspacevariant -lutil -lcomponent -lenvision -lgist -limage -lmodelneuron -lobjrec -lsift -lsurprise -lvfat 

# Required default libraries to comunicate with Carmen Core
LFLAGS += 
CFLAGS += -Wno-unused-parameter -Wno-ignored-qualifiers
CXXFLAGS += -Wno-unused-parameter -Wno-ignored-qualifiers

# Source code files (.c, .cpp) 
SOURCES = CcodeWriter.C DeBayer.C DeBayerREG.C DpxFile.C DpxParser.C GenericFrame.C JpegParser.C JpegWriter.C PfmParser.C PfmWriter.C \
PfzParser.C PfzWriter.C PlaintextWriter.C PngParser.C PngWriter.C PnmParser.C PnmWriter.C QuartzQuickTimeParser.C Raster.C \
RasterFileFormat.C RasterParser.C RasterWriter.C RawWriter.C YuvParser.C YuvWriter.C

#SUBDIRS = Component Media Transport Video ImageNeuro Raster Simulation Util nub rutz

TARGETS = libraster.a

libraster.a: CcodeWriter.o DeBayer.o DeBayerREG.o DpxFile.o DpxParser.o GenericFrame.o JpegParser.o JpegWriter.o PfmParser.o PfmWriter.o \
PfzParser.o PfzWriter.o PlaintextWriter.o PngParser.o PngWriter.o PnmParser.o PnmWriter.o QuartzQuickTimeParser.o Raster.o \
RasterFileFormat.o RasterParser.o RasterWriter.o RawWriter.o YuvParser.o YuvWriter.o

# rules

include $(CARMEN_HOME)/src/Makefile.rules
