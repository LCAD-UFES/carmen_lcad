include ../../Makefile.conf

MODULE_NAME = tracker_opentld/libopentld
MODULE_COMMENT = tracker_opentld/libopentld

LINK = g++

CXXFLAGS += 
CFLAGS += 
IFLAGS += `pkg-config --cflags opencv` -I ../3rdparty/cvblobs -I mftracker -I tld 
LFLAGS += `pkg-config --libs opencv`
SUBDIRS += 

SOURCES =  imacq/ImAcq.cpp \
    mftracker/BB.cpp \
    mftracker/BBPredict.cpp \
    mftracker/FBTrack.cpp \
    mftracker/Lk.cpp \
    mftracker/Median.cpp \
    tld/Clustering.cpp \
    tld/DetectionResult.cpp \
    tld/DetectorCascade.cpp \
    tld/EnsembleClassifier.cpp \
    tld/ForegroundDetector.cpp \
    tld/MedianFlowTracker.cpp \
    tld/NNClassifier.cpp \
    tld/TLD.cpp \
    tld/TLDUtil.cpp \
    tld/VarianceFilter.cpp
    
PUBLIC_INCLUDES = 
    
PUBLIC_LIBRARIES = libopentld.a
TARGETS = libopentld.a
PUBLIC_BINARIES = 

libopentld.a: imacq/ImAcq.o \
    mftracker/BB.o \
    mftracker/BBPredict.o \
    mftracker/FBTrack.o \
    mftracker/Lk.o \
    mftracker/Median.o \
    tld/Clustering.o \
    tld/DetectionResult.o \
    tld/DetectorCascade.o \
    tld/EnsembleClassifier.o \
    tld/ForegroundDetector.o \
    tld/MedianFlowTracker.o \
    tld/NNClassifier.o \
    tld/TLD.o \
    tld/TLDUtil.o \
    tld/VarianceFilter.o

include ../../Makefile.rules
