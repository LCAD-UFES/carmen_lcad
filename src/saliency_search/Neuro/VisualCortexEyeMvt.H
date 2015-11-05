/*!@file Neuro/VisualCortexEyeMvt.H a fake VisualCortex whose saliency map
  is built from human eye movement traces */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/VisualCortexEyeMvt.H $
// $Id: VisualCortexEyeMvt.H 11647 2009-09-03 01:18:22Z itti $
//

#ifndef VISUALCORTEXEYEMVT_H_DEFINED
#define VISUALCORTEXEYEMVT_H_DEFINED

#include "Image/ImageSet.H"
#include "Image/LevelSpec.H"
#include "Neuro/VisualCortex.H"
#include "Simulation/SimEvents.H"
#include <vector>

class SimTime;
class EyeTrace;

// ######################################################################
//! The VisualCortexEyeMvt Class
/*! This is a fake VisualCortex that outputs a saliency map built from
  a number of human eye movement traces. It contains an
  EyeTrackerEyeHeadController object that will provide eye position
  of the various humans, and it builds some sort of heat map from
  those. */
class VisualCortexEyeMvt : public VisualCortex
{
public:
  //! Construct with no channel; channels are ignored anyways
  VisualCortexEyeMvt(OptionManager& mgr,
                     const std::string& descrName = "Visual Cortex Eye Mvt",
                     const std::string& tagName = "VisualCortexEyeMvt");

  //! Virtual destructor for safe inheritance.
  virtual ~VisualCortexEyeMvt();

  //! Get the output from visual cortex.
  virtual Image<float> getOutput();

protected:
  //! Callback for every clock tick, to run our plotting
  SIMCALLBACK_DECLARE(VisualCortexEyeMvt, SimEventClockTick);

  //! Callback for every input frame
  SIMCALLBACK_DECLARE(VisualCortexEyeMvt, SimEventRetinaImage);

  //! Callback for every time we should save our outputs
  SIMCALLBACK_DECLARE(VisualCortexEyeMvt, SimEventSaveOutput);

  //! Callback for every time someone wants our features
  SIMREQHANDLER_DECLARE(VisualCortexEyeMvt, SimReqVCXfeatures);

  //! Callback for every time someone wants our maps
  SIMREQHANDLER_DECLARE(VisualCortexEyeMvt, SimReqVCXmaps);

  OModelParam<std::string> itsFnames; //!< names of eye movement files
  OModelParam<float> itsSigma; //!< sigma of gaussian blobs in pix at sml
  OModelParam<float> itsForgetFac; //!< forgetting factor
  OModelParam<uint> itsDelay; //!< human-to-human delay, in eye samples
  OModelParam<bool> itsUseMax; //!< use max to combine instead of sum
  OModelParam<bool> itsSaccadeOnly; //!< plot only when a saccade
  OModelParam<LevelSpec> itsLevelSpec; //!< our levelspec
  OModelParam<bool> itsSaveOutput;      //!< save our output?
  OModelParam<float> itsOutputFactor;   //!< output range factor

  // get started
  virtual void start1();

private:
  ImageSet<float> itsMaps;                               // one map per human
  std::vector<rutz::shared_ptr<EyeTrace> > itsEyeTrace;  // one per human
  std::vector<size_t> itsEyeSample;                      // one per human
  Image<float> itsOutputCache;                           // cached output map
};

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
