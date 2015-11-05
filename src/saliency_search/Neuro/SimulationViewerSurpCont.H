/*!@file Neuro/SimulationViewerSurpCont.H entry interface between INVT and ASAC */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
// by the University of Southern California (USC) and the iLab at USC.  //
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
// Primary maintainer for this file: T. Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerSurpCont.H $
// $Id: SimulationViewerSurpCont.H 10714 2009-02-01 07:16:36Z itti $
//

#ifndef SIMULATIONVIEWERSURPCONT_H_DEFINED
#define SIMULATIONVIEWERSURPCONT_H_DEFINED

#include "Neuro/ScaleSurpriseControl.H"

#include "Component/ModelParam.H"
#include "Image/ArrayData.H"   // for Dims
#include "Image/Image.H"       // for Image
#include "Image/ImageSet.H"
#include "Image/Pixels.H"      // for PixRGB<byte>
#include "Image/Point2DT.H"    // for Point2DT
#include "Media/MediaSimEvents.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/SimulationViewer.H"
#include "Neuro/SpatialMetrics.H"
#include "Neuro/VisualCortex.H"
#include "Neuro/WTAwinner.H"   // for WTAwinner
#include "Util/SimTime.H"
#include "rutz/shared_ptr.h"   // for rutz::shared_ptr
#include <fstream>

class Brain;
class FrameOstream;

//! This class provides an interface into SurpriseControl (ASAC)
/*! Surprise Control takes in an image and a set of final stage conspicuity
    images at different scales and uses them to drive different filters
    that are designed to reduce/control surprise in an image.

    In general, any type of conspicuity map can be used, but it's primarily
    designed to work with Surprise images. However, this version requires
    the standard default scales and channels (rg,by... etc). It does not
    know how to handle different features such as junctions.

    You need to supply:
    - A FrameSeries of raw images one at a time (per usual)
    - A configFile , there should be a default in etc
    - A brain , this is needed to extract the conspicuity maps

    This class will return:
    - A surprise controlled image
    - A difference image showing what it changed
    - A beta image showing its internal temporal smoothed surprise maps

    T. Nathan Mundhenk <mundhenk@usc.edu>
*/
class SimulationViewerSurpCont : public SimulationViewer {
public:
  // ######################################################################
  /*! @name Constructors and destructors */
  //@{

  //! Constructor. See ModelComponent.H.
  /*! @param mgr our ModelManager (see ModelManager.H)
      @param descrName descriptive name for human usage
      @param tagName name for ParamMap usage */
  SimulationViewerSurpCont(OptionManager& mgr,
                       const std::string& descrName = "ASAC Surprise Control",
                       const std::string& tagName   = "SurpriseControl");
  //! Destructor
  virtual ~SimulationViewerSurpCont();

  //! Set our brain
  virtual void setBrain(Brain* brain);

  //@}

  //! Initialize everything. Input the raw image size
  void init(const ushort baseSizeX,const ushort baseSizeY);

protected:
  //! Callback for when a new input frame is available
  SIMCALLBACK_DECLARE(SimulationViewerSurpCont, SimEventInputFrame);

  //! Callback for every clock tick
  SIMCALLBACK_DECLARE(SimulationViewerSurpCont, SimEventClockTick);

  //! Callback for every time we should save our outputs
  SIMCALLBACK_DECLARE(SimulationViewerSurpCont, SimEventSaveOutput);

  //! Save our various results
  virtual void save1(const ModelComponentSaveInfo& sinfo);

  //! save results
  void saveResults(const nub::ref<FrameOstream>& ofs);

  //! return the surprise controlled image
  Image<PixRGB<float> > getResult();

  //! return the difference images if needed
  std::vector<Image<PixRGB<float> > > getDiffImages();

  //! return the beta images if needed
  std::vector<Image<float> > getBetaImages();

protected:

  //! Draw current time onto given image
  void drawTime(Image< PixRGB<byte> >& image);

  //! metrics that depend on the input size:
  nub::ref<SpatialMetrics> itsMetrics;

  //! Should we output the difference image between the pre/post image
  OModelParam<bool> itsDrawDiffParts;
  //! should we output the surprise maps used here?
  OModelParam<bool> itsDrawBetaParts;
  //! should we output the bias maps used here?
  OModelParam<bool> itsDrawBiasParts;
  //! should we output the seperable filter layers used here?
  OModelParam<bool> itsDrawSeperableParts;
  //! config file to open
  OModelParam<std::string> itsConfigFile;
  //! LevelSpec used by our channels, used to compute output dims
  OModelParam<LevelSpec> itsLevelSpec;

private:
  SimTime   itsCurrTime;       // current time
  ScaleSurpriseControl<float> itsScaleSurpriseControl;
  //! local copy of the input image
  Image<PixRGB<float> > itsInput;
  //! input changed since last getTraj()
  bool      itsHasNewInput;
  //! tells us if we need to initialize ScaleSurpriseControl
  bool      itsInit;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
