include $(CARMEN_HOME)/src/Makefile.conf

# Module name and description
MODULE_NAME = ILab Saliency Library - Image
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
-lqtutil -lrutz -lspacevariant -lutil -lcomponent -lenvision -lgist -lmodelneuron -lobjrec -lraster -lsift -lsurprise -lvfat

# Required default libraries to comunicate with Carmen Core
LFLAGS += 
CFLAGS += -std=c99 -Wno-unused-parameter -Wno-ignored-qualifiers
CXXFLAGS += -Wno-unused-parameter -Wno-ignored-qualifiers

# Source code files (.c, .cpp) 
SOURCES = ArrayData.C ColorMap.C  ColorOps.C Convolutions.C CutPaste.C Dims.C DrawOps.C FFTWWrapper.C FilterOps.C FourierEngine.C \
IO.C ImageCache.C ImageSetOps.C IntegerMathOps.C JPEGUtil.C Kernels.C LevelSpec.C LogPolarTransform.C LowPass.C LowPassLpt.C MathOps.C \
MatrixOps.C MorphOps.C Normalize.C OpenCVUtil.C Pixels.C Point2D.C Point3D.C PyrBuilder.C PyramidCache.C PyramidOps.C \
PyramidTypes.C Range.C Rectangle.C ResizeSpec.C ShapeOps.C SimpleFont.C Transforms.C c_integer_math_ops.c color_conversions.C \
fancynorm.C lapack.C

#SUBDIRS = Component Media Transport Video ImageNeuro Raster Simulation Util nub rutz

TARGETS = libimage.a

libimage.a: ArrayData.o ColorMap.o  ColorOps.o Convolutions.o CutPaste.o Dims.o DrawOps.o FFTWWrapper.o FilterOps.o FourierEngine.o \
IO.o ImageCache.o ImageSetOps.o IntegerMathOps.o JPEGUtil.o Kernels.o LevelSpec.o LogPolarTransform.o LowPass.o LowPassLpt.o MathOps.o \
MatrixOps.o MorphOps.o Normalize.o OpenCVUtil.o Pixels.o Point2D.o Point3D.o PyrBuilder.o PyramidCache.o PyramidOps.o \
PyramidTypes.o Range.o Rectangle.o ResizeSpec.o ShapeOps.o SimpleFont.o Transforms.o c_integer_math_ops.o color_conversions.o \
fancynorm.o lapack.o

# rules

include $(CARMEN_HOME)/src/Makefile.rules
