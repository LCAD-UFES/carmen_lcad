/*!@file ModelNeuron/FreeViewingModel.H */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/FreeViewingModel.H $

#ifndef FREEVIEWINGMODEL_H_DEFINED
#define FREEVIEWINGMODEL_H_DEFINED

#include "ModelNeuron/SimulationWorkQueue.H"
#include "ModelNeuron/FreeViewingData.H"
#include "ModelNeuron/SpatioTemporalEngine.H"
#include "ModelNeuron/FreeViewingChannels.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/SimulationPlot1D.H"
#include "Simulation/SimEvents.H"
#include "Simulation/SimModule.H"
#include "Media/MgzEncoder.H"
#include "Media/MgzDecoder.H"

// ######################################################################
//! The FreeViewingModel class
class FreeViewingModel : public SimModule
{
public:
  // ######################################################################
  /*! @name Constructors/Destructor */
  //@{

  //! Constructor
  /*! @param mgr our ModelManager (see ModelManager.H)
      @param descrName descriptive name for human usage
      @param tagName name for ParamMap usage */
    FreeViewingModel(OptionManager& mgr,
                     const std::string& descrName = "Free Viewing Model",
                     const std::string& tagName = "FreeViewingModel");
    
    //! Virtual destructor for safe inheritance.
    virtual ~FreeViewingModel();
    
    //@}
    
  protected:
    //! Callback for when a new input (retina) frame is available
    SIMCALLBACK_DECLARE(FreeViewingModel, SimEventRetinaImage);
    
    //! Callback for every time we should save our outputs
    SIMCALLBACK_DECLARE(FreeViewingModel, SimEventSaveOutput);
    
    //the size at which we pool over visual cortex units (in degrees).
    OModelParam<Dims> itsSCPoolingSize;

    //! maxnorm params
    OModelParam<float> itsSCChannelMaxNormEsig;
    OModelParam<float> itsSCChannelMaxNormIsig;
    OModelParam<float> itsSCChannelMaxNormStrength;

    //! maxnorm params
    OModelParam<float> itsSCSCMaxNormEsig;
    OModelParam<float> itsSCSCMaxNormIsig;
    OModelParam<float> itsSCSCMaxNormStrength;

    //! norm by sum of channels?
    OModelParam<bool> itsSCChanNormSum;

    //!norm by max output over all clips?
    OModelParam<bool> itsSCChanNormMax;

    //!only process channels
    OModelParam<bool> itsSCChannelsOnly;

    //demo retinal display size
    OModelParam<Dims> itsRetinalDisplaySize;

    //should we save channel output?
    OModelParam<bool> itsDisplayChan;

    //should we save sc inverse?
    OModelParam<bool> itsSCInverse;

    //should we save sc demo?
    OModelParam<bool> itsSCDemo;

    //map factor
    OModelParam<float> itsMapFac;

    //norm file name
    OModelParam<std::string> itsNormFileName;

    //update norm file
    OModelParam<bool> itsUpdateNormFile;

    //update norm file
    OModelParam<std::string> itsLoadChanFile;

    //update norm file
    OModelParam<std::string> itsSaveChanFile;

    //probe location
    OModelParam<Point2D<float> > itsStimLocation;

    //ppd
    OModelParam<PixPerDeg> itsPixPerDeg;

    //framing image for size
    OModelParam<std::string> itsFramingImageName;

    //channel output file
    OModelParam<std::string> itsOutputFileBase;

    //neural data filename
    OModelParam<std::string> itsNeuralFileName;

    //sampling rate of data
    OModelParam<uint> itsDownSampleFactor;

    //sampling rate of data
    NModelParam<SimTime> itsSamplingRate;

    //!start things up
    void start1();

    //!start things up
    void start2();

    //!stop things
    void stop2();
    
  private:
    FreeViewingModel(const FreeViewingModel&); // not allowed
    FreeViewingModel& operator=(const FreeViewingModel&); // not allowed

    //the sc rf kernel
    Image<float> createSCRfKernel();

    //get points around pos in map with size dims
    std::vector<Point2D<int> > getMapPoints(Dims const & dims, Point2D<int> const & pos);
    
    //sample poionts from a map
    float sampleFromMap(Image<float> const & map, std::vector<Point2D<int> > const & pnts);  

    nsu::SimulationWorkQueue itsWorker;

    uint frame;
    uint downsampleFac;
    Dims itsRetinalDisplaySizeLoc;
    bool doNorm;
    float itsSCChanMaxNormEsig;
    float itsSCChanMaxNormIsig;
    float itsSCChanMaxNormStrength;
    float itsSCMaxNormEsig;
    float itsSCMaxNormIsig;
    float itsSCMaxNormStrength;
    bool itsChanNormSum;
    bool itsChanNormMax;
    bool itsOnlyChan;
    Point2D<float> itsStimLoc;
    bool itsSaveResults;
    float itsMapFactor;
    Image<float> itsSCRf;               //kernel describing SC RF Structure.
    Image<PixRGB<byte> > itsRetinal;    //retinal frame shifted to current eye position
    Image<float> itsSCInput;            //the input to the sc model
    Image<float> itsSC;                 //the sc response
    ImageSet<float> itsChanOutputs;     //output of each channel
    std::vector<Range<float> > itsChanRange; // hold any channel range data
    SimulationPlot1D itsPlot, itsNeuralPlot;
    std::vector<float> itsNeuralData;

    rutz::shared_ptr<SpatioTemporalEnergyEngine> itsSpeEngine;
    rutz::shared_ptr<MgzDecoder> itsChanDecoder;
    rutz::shared_ptr<MgzEncoder> itsChanEncoder;

    std::vector<Point2D<int> > itsSamplePoints;
    std::vector<float> itsSalResults;
    std::vector<std::vector<float> > itsChanResults;
    
    // channel info
    std::vector<std::string> itsChannelNames;
    std::vector<nub::soft_ref<FreeViewingChannel> > itsChannels;
};

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
