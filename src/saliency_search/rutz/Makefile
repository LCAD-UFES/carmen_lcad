include $(CARMEN_HOME)/src/Makefile.conf

# Module name and description
MODULE_NAME = ILab Saliency Library - rutz
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
-lqtutil -lspacevariant -lutil -lcomponent -lenvision -lgist -limage -lmodelneuron -lobjrec -lraster -lsift -lsurprise -lvfat

# Required default libraries to comunicate with Carmen Core
LFLAGS += 
CFLAGS += -Wno-unused-parameter -Wno-ignored-qualifiers
CXXFLAGS += -Wno-unused-parameter -Wno-ignored-qualifiers

# Source code files (.c, .cpp) 
SOURCES = abort.cc assocarray.cc backtrace.cc bzip2stream.cc compat_cmath.cc debug.cc demangle.cc error.cc error_context.cc factory.cc \
freelist.cc fstring.cc gzstreambuf.cc iter.cc mappedfile.cc mutex.cc pipe.cc prof.cc rand.cc sfmt.cc shared_ptr.cc stderror.cc stdiobuf.cc \
time.cc timeformat.cc trace.cc unixcall.cc

#SUBDIRS = Component Media Transport Video ImageNeuro Raster Simulation Util nub rutz

TARGETS = librutz.a

librutz.a: abort.o assocarray.o backtrace.o bzip2stream.o compat_cmath.o debug.o demangle.o error.o error_context.o factory.o \
freelist.o fstring.o gzstreambuf.o iter.o mappedfile.o mutex.o pipe.o prof.o rand.o sfmt.o shared_ptr.o stderror.o stdiobuf.o \
time.o timeformat.o trace.o unixcall.o

# rules

include $(CARMEN_HOME)/src/Makefile.rules
