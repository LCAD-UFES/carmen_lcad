/*!@file Neuro/SimulationViewerEyeMvt2.H comparison between saliency and
  human eye movements */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerEyeMvt2.H $
// $Id: SimulationViewerEyeMvt2.H 10825 2009-02-11 07:46:59Z itti $
//

#ifndef SIMULATIONVIEWEREYEMVT2_H_DEFINED
#define SIMULATIONVIEWEREYEMVT2_H_DEFINED

#include "Component/ModelParam.H"
#include "Image/ImageCache.H"
#include "Image/ImageSet.H"
#include "Image/LevelSpec.H"
#include "Media/MediaSimEvents.H"
#include "Neuro/SimulationViewer.H"
#include "Neuro/NeuroSimEvents.H"
#include "Simulation/SimEvents.H"
#include "Util/SimTime.H"

class EyeTrackerSaccadeController;
class SpatialMetrics;
class VisualBuffer;

//! Measure salience at human eye positions
class SimulationViewerEyeMvt2 : public SimulationViewer {
public:
  // ######################################################################
  /*! @name Constructors and destructors */
  //@{

  //! Constructor. See ModelComponent.H.
  SimulationViewerEyeMvt2(OptionManager& mgr,
                          const std::string& descrName =
                          "EyeMvt2 Simulation Viewer",
                          const std::string& tagName =
                          "SimulationViewerEyeMvt2");

  //! Destructor
  virtual ~SimulationViewerEyeMvt2();

  //@}

protected:
  //! Callback for when the head moves
  SIMCALLBACK_DECLARE(SimulationViewerEyeMvt2, SimEventInputFrame);

  //! Callback for when a new retina image is available
  SIMCALLBACK_DECLARE(SimulationViewerEyeMvt2, SimEventRetinaImage);

  //! Callback for when a new attention shift occurs
  SIMCALLBACK_DECLARE(SimulationViewerEyeMvt2, SimEventWTAwinner);

  //! Callback for when the eye moves
  SIMCALLBACK_DECLARE(SimulationViewerEyeMvt2, SimEventSaccadeStatusEye);

  //! Callback for every time we should save our outputs
  SIMCALLBACK_DECLARE(SimulationViewerEyeMvt2, SimEventSaveOutput);

  //! Get the attention/eye/head trajectory image
  Image< PixRGB<byte> > getTraj(SimEventQueue& q);

  //@}

  nub::ref<SpatialMetrics> itsMetrics;  //!< metrics that depend on input size
  OModelParam<bool> itsSaveMegaCombo;       //!< save mega combo?
  NModelParam< PixRGB<byte> > itsColorEye;  //!< patch color for human eye mvts
  OModelParam<std::string> itsOutFname;     //!< Name of output file
  OModelParam<LevelSpec> itsLevelSpec;      //!< levelspec for buffer scale
  OModelParam<bool> itsUseIOR;              //!< try to use IOR

  //! Get started
  virtual void start1();

  //! get stopped
  virtual void stop1();

private:
  Image< PixRGB<byte> > itsTraj;         // trajectory
  Image< PixRGB<byte> > itsRawTraj;      // raw unshifted trajectory
  FILE *itsOutFile;
  Point2D<int> itsLastSample;
  int itsLastRadius;

  Image<byte> itsIORmask;
  bool itsDidAttentionShift;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
