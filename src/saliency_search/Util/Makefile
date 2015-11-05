include $(CARMEN_HOME)/src/Makefile.conf

# Module name and description
MODULE_NAME = ILab Saliency Library - Util
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

LFLAGS += -lbeowulf -lgui -lneuro -lpsycho -lrobots -lsimulation -ltransport -lvideo -lchannels -ldevices -lhmax -llearn -lmedia -lnub \
-lqtutil -lrutz -lspacevariant -lcomponent -lenvision -lgist -limage -lmodelneuron -lobjrec -lraster -lsift -lsurprise -lvfat
# Required default libraries to comunicate with Carmen Core
LFLAGS += 
CFLAGS += -Wno-unused-parameter -Wno-ignored-qualifiers
CXXFLAGS += -Wno-unused-parameter -Wno-ignored-qualifiers

# Source code files (.c, .cpp) 
SOURCES = Alloc.C AllocAux.C ByteCount.C ErrorStrategy.C FileUtil.C JobServer.C JobWithSemaphore.C MainJobServer.C MathFunctions.C \
Pause.C SimTime.C StringConversions.C StringUtil.C SyncJobServer.C TextLog.C WorkThreadServer.C csignals.C fpe.C fpu.C jenkinshash.C \
log.C readConfig.C sformat.C terminate.C version.C

#SUBDIRS = Component Media Transport Video ImageNeuro Raster Simulation Util nub rutz

TARGETS = libutil.a

libutil.a: Alloc.o AllocAux.o ByteCount.o ErrorStrategy.o FileUtil.o JobServer.o JobWithSemaphore.o MainJobServer.o MathFunctions.o \
Pause.o SimTime.o StringConversions.o StringUtil.o SyncJobServer.o TextLog.o WorkThreadServer.o csignals.o fpe.o fpu.o jenkinshash.o \
log.o readConfig.o sformat.o terminate.o version.o

# rules

include $(CARMEN_HOME)/src/Makefile.rules
