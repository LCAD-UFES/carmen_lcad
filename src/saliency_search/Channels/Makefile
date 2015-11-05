include $(CARMEN_HOME)/src/Makefile.conf

# Module name and description
MODULE_NAME = ILab Saliency Library - Channels
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

LFLAGS += -lbeowulf -lgui -lneuro -lpsycho -lrobots -lsimulation -ltransport -lvideo -ldevices -lhmax -llearn -lmedia -lnub \
-lqtutil -lrutz -lspacevariant -lutil -lcomponent -lenvision -lgist -limage -lmodelneuron -lobjrec -lraster -lsift -lsurprise -lvfat
# Required default libraries to comunicate with Carmen Core
CFLAGS += -std=c99 -Wall
CXXFLAGS += -Wno-unused-parameter -Wno-ignored-qualifiers

# Source code files (.c, .cpp) 
SOURCES = BlueChannel.C BlueYellowChannel.C CIELabChannel.C ChannelBase.C ChannelFacet.C ChannelFacetScalar.C ChannelFacets.C \
ChannelMaps.C ChannelOpts.C ChannelVisitor.C ColorBandChannel.C ColorChannel.C ColorComputeTypes.C ComplexChannel.C \
CompositeColorChannel.C ContourChannel.C DKLTypes.C DKLcolorChannel.C DcolorChannel.C DepthChannel.C DepthMotionChannel.C \
DirectionChannel.C DirectionOpticalFlowChannel.C DirectionSpatioTemporalChannel.C DummyChannel.C EndPointChannel.C \
EntropyChannel.C FlickerChannel.C FlickerNBackChannel.C FoeMSTChannel.C ForegroundDetectionChannel.C GaborChannel.C \
GaussianPyrChannel.C GreenChannel.C GreenRedChannel.C GuidedSearch.C H2SVChannel.C Hue1Channel.C Hue2Channel.C HueChannel.C \
ImagizeAlphaChannel.C ImagizeBetaChannel.C ImagizeColorChannel.C ImagizeLedChannel.C InformationChannel.C InputFrame.C \
InputHandler.C InputHandlerThreaded.C IntegerChannel.C IntegerColorChannel.C IntegerComplexChannel.C IntegerDecodeType.C \
IntegerDirectionChannel.C IntegerFlickerChannel.C IntegerInput.C IntegerIntensityChannel.C IntegerMathEngine.C IntegerMotionChannel.C \
IntegerOrientationChannel.C IntegerRawVisualCortex.C IntegerSimpleChannel.C IntensityBandChannel.C IntensityChannel.C Jet.C \
JunctionChannel.C KcolorChannel.C LJunctionChannel.C LcolorChannel.C MSTChannel.C MapCombineType.C MichelsonChannel.C MotionChannel.C \
MotionOpticalFlowChannel.C MotionSpatioTemporalChannel.C MultiColorBandChannel.C MultiSpectralResidualChannel.C ObjDetChannel.C \
OptimalGains.C OrientComputeTypes.C OrientationChannel.C PN03contrastChannel.C PedestrianChannel.C RawVisualCortex.C RedChannel.C \
RedGreenChannel.C SIFTChannel.C SOBlueYellowChannel.C SOColorChannel.C SOGreenRedChannel.C SORedGreenChannel.C SOYellowBlueChannel.C \
SaturationChannel.C ScorrChannel.C SingleChannel.C SingleOpponentChannel.C SkinHueChannel.C SoxChannel.C SpectralResidualChannel.C \
SubmapAlgorithm.C SubmapAlgorithmBiased.C SubmapAlgorithmStd.C TJunctionChannel.C TcorrChannel.C TemplateMatchChannel.C \
ValueIntensityChannel.C VarianceChannel.C VisualFeatures.C XJunctionChannel.C YellowBlueChannel.C YellowChannel.C ZeroChannel.C

#SUBDIRS = Component Media Transport Video ImageNeuro Raster Simulation Util nub rutz

TARGETS = libchannels.a

libchannels.a: BlueChannel.o BlueYellowChannel.o CIELabChannel.o ChannelBase.o ChannelFacet.o ChannelFacetScalar.o ChannelFacets.o \
ChannelMaps.o ChannelOpts.o ChannelVisitor.o ColorBandChannel.o ColorChannel.o ColorComputeTypes.o ComplexChannel.o \
CompositeColorChannel.o ContourChannel.o DKLTypes.o DKLcolorChannel.o DcolorChannel.o DepthChannel.o DepthMotionChannel.o \
DirectionChannel.o DirectionOpticalFlowChannel.o DirectionSpatioTemporalChannel.o DummyChannel.o EndPointChannel.o \
EntropyChannel.o FlickerChannel.o FlickerNBackChannel.o FoeMSTChannel.o ForegroundDetectionChannel.o GaborChannel.o \
GaussianPyrChannel.o GreenChannel.o GreenRedChannel.o GuidedSearch.o H2SVChannel.o Hue1Channel.o Hue2Channel.o HueChannel.o \
ImagizeAlphaChannel.o ImagizeBetaChannel.o ImagizeColorChannel.o ImagizeLedChannel.o InformationChannel.o InputFrame.o \
InputHandler.o InputHandlerThreaded.o IntegerChannel.o IntegerColorChannel.o IntegerComplexChannel.o IntegerDecodeType.o \
IntegerDirectionChannel.o IntegerFlickerChannel.o IntegerInput.o IntegerIntensityChannel.o IntegerMathEngine.o IntegerMotionChannel.o \
IntegerOrientationChannel.o IntegerRawVisualCortex.o IntegerSimpleChannel.o IntensityBandChannel.o IntensityChannel.o Jet.o \
JunctionChannel.o KcolorChannel.o LJunctionChannel.o LcolorChannel.o MSTChannel.o MapCombineType.o MichelsonChannel.o MotionChannel.o \
MotionOpticalFlowChannel.o MotionSpatioTemporalChannel.o MultiColorBandChannel.o MultiSpectralResidualChannel.o ObjDetChannel.o \
OptimalGains.o OrientComputeTypes.o OrientationChannel.o PN03contrastChannel.o PedestrianChannel.o RawVisualCortex.o RedChannel.o \
RedGreenChannel.o SIFTChannel.o SOBlueYellowChannel.o SOColorChannel.o SOGreenRedChannel.o SORedGreenChannel.o SOYellowBlueChannel.o \
SaturationChannel.o ScorrChannel.o SingleChannel.o SingleOpponentChannel.o SkinHueChannel.o SoxChannel.o SpectralResidualChannel.o \
SubmapAlgorithm.o SubmapAlgorithmBiased.o SubmapAlgorithmStd.o TJunctionChannel.o TcorrChannel.o TemplateMatchChannel.o \
ValueIntensityChannel.o VarianceChannel.o VisualFeatures.o XJunctionChannel.o YellowBlueChannel.o YellowChannel.o ZeroChannel.o

# rules

include $(CARMEN_HOME)/src/Makefile.rules
