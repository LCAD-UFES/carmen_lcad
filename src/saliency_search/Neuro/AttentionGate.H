/*!@file Neuro/AttentionGuidanceMap.H Class declarations for task-relevance map class */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/AttentionGate.H $
// $Id: AttentionGate.H 10794 2009-02-08 06:21:09Z itti $
//

#ifndef ATTENTIONGATE_H_DEFINED
#define ATTENTIONGATE_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Media/MediaSimEvents.H"
#include "Neuro/NeuroSimEvents.H"
#include "Simulation/SimModule.H"
#include "Simulation/SimEvents.H"
#include "Util/SimTime.H"
#include "VFAT/segmentImageMC2.H"

enum AG_METHODS_ENUM
{
  AG_CENTER,
  AG_MAX,
  AG_NORMALIZE,
};

class FrameOstream;
class ModelManager;
class SimTime;
class VisualCortex;

// ######################################################################
//! The Attention Gate Class
// ######################################################################
/*! This is a class that models the first and second stage of the two stage
    attention gating mechanism. The first stage decides what visual information
    will get through to the next stage. The second stage is not so much a gate
    as it is an integrator.

    Stage One: The first stage must account for both attention blocking and
               attention capture. So for each frame, what parts of the image
               get through is effected by frames that come before as well
               as afterwards. So, in order for something to get through it must
               be more powerful than something that comes in the next image
               (Blocking) and more powerful than something that came before it
               (Capture).

               Simple: The simple model uses the basic attention guidance
                       map as the basis for blocking. This has the advantage
                       of simplicity, but has the drawback that channels
                       seem to act at different time scales and interact
                       in special ways.

              Complex: The complex model allows different channels to dwell
                       for longer or shorter time intervals. For instance,
                       it seems that color should have a strong short period
                       of action, but that luminance and orientations have a
                       longer but less pronounced dwell time. Also, orientation
                       channels may have orthogonal interactions.

    Stage Two: The second stage is an integrator. Images that come first in
               a series begin to assemble. If another image comes in with a
               set of similar features it aids in its coherence. However,
               if the first image is more coherent it will absorb the first
               image. As such a frame can block a frame that follows at
               100-300 ms by absorbing it. If this happens the first image
               is enhanced by absorbing the second image. The second stage
               emulates the resonable expectation of visual flow in a sequence
               of images, but allows for non fluid items to burst through.

    The AG is based on the outcome from our recent RSVP work as well as work
    by Sperling et al 2001 and Chun and Potter 1995.

. */
class AttentionGate : public SimModule
{
public:
  // ######################################################################
  //! @name Constructor, destructor
  //@{

  //! Ininitialized constructor
  /*! The map will be resized and initialized the first time input() is
    called */
  AttentionGate(OptionManager& mgr,
                const std::string& descrName = "Attention Gate Map",
                const std::string& tagName = "AttentionGate",
                const nub::soft_ref<VisualCortex> vcx =
                nub::soft_ref<VisualCortex>());

  //! Destructor
  virtual ~AttentionGate();

  //@}

protected:
  //! Save our internals when saveResults() is called?
  OModelParam<bool> itsSaveResults;
  nub::soft_ref<VisualCortex> itsVCX;
  unsigned short itsSizeX;
  unsigned short itsSizeY;
  unsigned int   itsFrameNumber;
  Image<PixRGB<float> > itsLastFrame;
  Image<PixRGB<float> > itsCurrFrame;
  float itsLogSigO;
  float itsLogSigS;

private:
  // forbid assignment and copy-construction:
  AttentionGate& operator=(const AttentionGate& sm);
  AttentionGate(const AttentionGate& sm);
};

// ######################################################################
//! AttentionGate configurator
// ######################################################################
class AttentionGateConfigurator : public ModelComponent
{
public:
  //! Constructor
  AttentionGateConfigurator(OptionManager& mgr,
                               const std::string& descrName =
                               "Attention Gate Configurator",
                               const std::string& tagName =
                               "AttentionGateConfigurator");

  //! destructor
  virtual ~AttentionGateConfigurator();

  //! Get the chosen AG
  /*! You should call this during start() of the ModelComponent that
      needs the AG.  */
  nub::ref<AttentionGate> getAG() const;

protected:
  OModelParam<std::string> itsAGtype; //!< type of map

  //! Intercept people changing our ModelParam
  /*! See ModelComponent.H; as parsing the command-line or reading a
    config file sets our name, we'll also here instantiate a
    controller of the proper type (and export its options) */
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

private:
  nub::ref<AttentionGate> itsAG; // the map
};



// ######################################################################
//! The standard attention gate
// ######################################################################
/*! This is our current standard AG implementation. */
class AttentionGateStd : public AttentionGate
{
public:
  //! Uninitialized constructor
  AttentionGateStd(OptionManager& mgr, const std::string& descrName =
                   "Task-Relevance Map Std",
                   const std::string& tagName =
                   "AttentionGateStd");

  //! Destructor
  virtual ~AttentionGateStd();

  //! Reset to initial state just after construction
  virtual void reset1();

protected:
  //! Callback for input frames
  SIMCALLBACK_DECLARE(AttentionGateStd, SimEventInputFrame);

  //! Callback for AGM
  SIMCALLBACK_DECLARE(AttentionGateStd, SimEventAttentionGuidanceMapOutput);

  //! Callback for every time we should save our outputs
  SIMCALLBACK_DECLARE(AttentionGateStd, SimEventSaveOutput);

  //! Compute for objects the max surprise location and value
  /*! For each potential object, find the most surprising region, its value
      and store its ID from segmentImageMC2. This will then be used in later
      methods to extract features from the objects.
  */
  virtual void computeMinMaxXY(const Image<float>& attentionMap,
                               const Image<int>&   segments,
                               const Image<bool>&  candidates);

  //! extract features from the surprising locations
  /*! For each segmented object, take the most surprising location and extract
      the features. When done store the objects and features in a deque. Each
      element in the deque corresponds to one frame
  */
  virtual void extractFeatureValues(SimEventQueue& q);

  //! Find the feature distance between objects
  /*! For each object in each frame, compute its feature distance from
      all other objects using a standard sum of squares distance.
  */
  virtual void computeFeatureDistance();

  virtual Image<float> getLastAttentionMap() const;

  virtual Image<float> getCurrentAttentionMap() const;

  //! Return all our values as an Image<float>
  virtual Image<float> getValue(SimEventQueue& q);

private:
  //! The first stage is a true gate. it returns a mask of what will get through
  /*! The simple form does not treat channels differently and only uses the
      general final attention map over all channels.
  */
  void stageOneGateSimple(SimEventQueue& q);
  //! The first stage is a true gate. it returns a mask of what will get through
  /*! The complex first gate interacts the channels and can also treat some
      channels with different time scales with different leaky integrator
      constants.  */
  void stageOneGateComplex(SimEventQueue& q);

  //! The second stage is an integrator which melds features across frames
  void stageTwoGate(SimEventQueue& q);

  bool itsSegmentDone;

  //! type
  OModelParam<std::string> itsAGStageOneType;
  OModelParam<std::string> itsAGStageTwoType;
  OModelParam<int>         itsAGStageTwoEpochs;
  uint                     itsTotalEpochs;
  uint                     itsMaxStageTwoFrames;
  AG_METHODS_ENUM          itsStageTwoGetFeatures;

  SimTime      itsT;                     // time of last integration
  SimTime      itsNewT;
  SimTime      itsTimeStep;
  float        itsC;
  float        itsLeak;
  Image<float> itsLastAttentionMap;      // contains both forward and backwards
  Image<float> itsCurrentAttentionMap;
  Image<float> itsStageOneGate;
  Image<float> itsStageTwoGate;
  Image<float> itsInput;
  Image<int>   itsStageTwoSegments;

  //! Stage Two object location list X position
  std::vector<int>   itsStageTwoObjectX;
  //! Stage Two object location list Y position
  std::vector<int>   itsStageTwoObjectY;
  //! Stage Two object surprise value
  std::vector<float> itsStageTwoObjectVal;
  //! Stage Two object ID list
  std::vector<int>   itsStageTwoObjectID;
  //! store the stage two objects over n frames
  std::deque<SimEventAttentionGateStageTwoObjects> itsStageTwoObjects;
  //! Object segmenter used in stage two
  segmentImageMC2<float,unsigned int,1> itsSegmenter;
};

// ######################################################################
//! The standard  map
// ######################################################################
/*! This is our current standard AG implementation. */
class AttentionGateStub : public AttentionGate
{
public:
  //! Uninitialized constructor
  AttentionGateStub(OptionManager& mgr, const std::string& descrName =
                    "Task-Relevance Map Std",
                    const std::string& tagName =
                    "AttentionGateStub");

  //! Destructor
  virtual ~AttentionGateStub();
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
