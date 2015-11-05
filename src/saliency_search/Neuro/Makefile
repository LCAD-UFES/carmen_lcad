include $(CARMEN_HOME)/src/Makefile.conf

# Module name and description
MODULE_NAME = ILab Saliency Library - Neuro
MODULE_COMMENT = Neural Visual Saliency Searcher

LINK = g++

# Application specific include directories.
IFLAGS += -I../ -I/usr/include/libxml2/ -include config.h -std=c++0x

LFLAGS += -L/usr/lib/x86_64-linux-gnu/ -L/usr/lib/i386-linux-gnu/ -L/usr/local/lib/ -L/usr/lib/libblas/

LFLAGS += -L./Beowulf -L./GUI -L./Neuro -L./Psycho -L./Robots -L./Simulation -L./Transport -L./Video -L./Channels -L./Devices \
-L./HMAX -L./Learn -L./Media -L./nub -L./QtUtil -L./rutz -L./SpaceVariant -L./Util -L./Component -L./Envision -L./Gist \
-L./Image -L./ModelNeuron -L./ObjRec -L./Raster -L./SIFT -L./Surprise -L./VFAT

LFLAGS += -lGL -lXtst -lXext -lX11 -lGLEW -lQtGui -lQtCore -lSDL -lSDL_gfx -lXtst -lXext -lX11 -lbz2 -lcv -lcxcore -lcvaux -lcxcore \
-lcv -lcvaux -ldc1394 -ldl -lglut -lGLU -lGL -lXtst -lXext -lX11 -ljpeg -llapack -lblas -lm -lpng -lpthread -lswscale -lavformat \
-lavcodec -lavutil -lpng -lz -lxml2 -lz

LFLAGS += -lbeowulf -lgui -lpsycho -lrobots -lsimulation -ltransport -lvideo -lchannels -ldevices -lhmax -llearn -lmedia -lnub \
-lqtutil -lrutz -lspacevariant -lutil -lcomponent -lenvision -lgist -limage -lmodelneuron -lobjrec -lraster -lsift -lsurprise -lvfat

CFLAGS += -Wno-unused-parameter -Wno-ignored-qualifiers
CXXFLAGS = -std=c++0x -D__GXX_EXPERIMENTAL_CXX0X_  -Wno-unused-parameter -Wno-ignored-qualifiers

# Source code files (.c, .cpp) 
SOURCES = AttentionGate.C AttentionGuidanceMap.C Brain.C EnvOpts.C EnvVisualCortex.C EyeHeadController.C EyeHeadControllerConfigurator.C \
EyeHeadControllers.C GistEstimator.C GistEstimatorBeyondBoF.C GistEstimatorConfigurator.C GistEstimatorContextBased.C GistEstimatorFFT.C \
GistEstimatorGen.C GistEstimatorStd.C GistEstimatorStub.C GistEstimatorSurfPMK.C GistEstimatorTexton.C HandController.C \
HandControllerConfigurator.C HandControllers.C IORtypes.C InferoTemporal.C InferoTemporalConfigurator.C InferoTemporalHmax.C \
InferoTemporalSIFT.C InferoTemporalSalBayes.C NeuroOpts.C NeuroSimEvents.C PrefrontalCortex.C Retina.C SVMClassifierModule.C \
SaccadeController.C SaccadeControllerConfigurator.C SaccadeControllers.C SaliencyMap.C SaliencyMapConfigurator.C SaliencyMapStdOptim.C \
ScaleSurpriseControl.C ShapeEstimator.C ShapeEstimatorModes.C SimulationPlot1D.C SimulationViewer.C SimulationViewerCompress.C \
SimulationViewerConfigurator.C SimulationViewerEyeHand.C SimulationViewerEyeMvt.C SimulationViewerEyeMvt2.C SimulationViewerEyeMvtNeuro.C \
SimulationViewerEyeRegion.C SimulationViewerEyeSim.C SimulationViewerHand.C SimulationViewerNerdCam.C SimulationViewerRecStats.C \
SimulationViewerStats.C SimulationViewerStd.C SimulationViewerStub.C SimulationViewerSurpCont.C SingleChannelBeo.C SpatialMetrics.C \
StdBrain.C TargetChecker.C TaskRelevanceMap.C VisualBuffer.C VisualCortex.C VisualCortexBeo.C VisualCortexConfigurator.C \
VisualCortexEyeMvt.C VisualCortexSurprise.C WinnerTakeAll.C WinnerTakeAllConfigurator.C WinnerTakeAllStdOptim.C

#SUBDIRS = Component Media Transport Video ImageNeuro Raster Simulation Util nub rutz

TARGETS = libneuro.a

libneuro.a: AttentionGate.o AttentionGuidanceMap.o Brain.o EnvOpts.o EnvVisualCortex.o EyeHeadController.o EyeHeadControllerConfigurator.o \
EyeHeadControllers.o GistEstimator.o GistEstimatorBeyondBoF.o GistEstimatorConfigurator.o GistEstimatorContextBased.o GistEstimatorFFT.o \
GistEstimatorGen.o GistEstimatorStd.o GistEstimatorStub.o GistEstimatorSurfPMK.o GistEstimatorTexton.o HandController.o \
HandControllerConfigurator.o HandControllers.o IORtypes.o InferoTemporal.o InferoTemporalConfigurator.o InferoTemporalHmax.o \
InferoTemporalSIFT.o InferoTemporalSalBayes.o NeuroOpts.o NeuroSimEvents.o PrefrontalCortex.o Retina.o SVMClassifierModule.o \
SaccadeController.o SaccadeControllerConfigurator.o SaccadeControllers.o SaliencyMap.o SaliencyMapConfigurator.o SaliencyMapStdOptim.o \
ScaleSurpriseControl.o ShapeEstimator.o ShapeEstimatorModes.o SimulationPlot1D.o SimulationViewer.o SimulationViewerCompress.o \
SimulationViewerConfigurator.o SimulationViewerEyeHand.o SimulationViewerEyeMvt.o SimulationViewerEyeMvt2.o SimulationViewerEyeMvtNeuro.o \
SimulationViewerEyeRegion.o SimulationViewerEyeSim.o SimulationViewerHand.o SimulationViewerNerdCam.o SimulationViewerRecStats.o \
SimulationViewerStats.o SimulationViewerStd.o SimulationViewerStub.o SimulationViewerSurpCont.o SingleChannelBeo.o SpatialMetrics.o \
StdBrain.o TargetChecker.o TaskRelevanceMap.o VisualBuffer.o VisualCortex.o VisualCortexBeo.o VisualCortexConfigurator.o \
VisualCortexEyeMvt.o VisualCortexSurprise.o WinnerTakeAll.o WinnerTakeAllConfigurator.o WinnerTakeAllStdOptim.o

# rules

include $(CARMEN_HOME)/src/Makefile.rules
