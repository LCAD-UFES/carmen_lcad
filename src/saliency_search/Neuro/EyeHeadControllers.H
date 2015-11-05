/*!@file Neuro/EyeHeadControllers.H Eye/Head controllers */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/EyeHeadControllers.H $
// $Id: EyeHeadControllers.H 10725 2009-02-02 01:12:31Z itti $
//

#ifndef NEURO_EYEHEADCONTROLLERS_H_DEFINED
#define NEURO_EYEHEADCONTROLLERS_H_DEFINED

#include "Neuro/EyeHeadController.H"
#include "Neuro/SaccadeControllers.H"
#include "Neuro/SaccadeControllerConfigurator.H"
#include "Simulation/SimEvents.H"

#include <vector>

class EyeTrace;

// ######################################################################
//! Stub eye/head controller
/*! No-op implementation -- never makes a decision, no matter what eye
  and head SaccadeControllers it may hold. */

class StubEyeHeadController : public EyeHeadController
{
public:
  //! Constructor
  StubEyeHeadController(OptionManager& mgr);

  //! Destructor
  virtual ~StubEyeHeadController();
};

// ######################################################################
//! Simple eye/head controller
/*! This fairly trivial controller uses two independent
  SaccadeController objects to move the eye and the head. There is
  basically no interaction between the two controllers, beyond the
  fact that covert attention feeds to the eye, which then generates
  overt attention shifts, which are fed to the head. */
class SimpleEyeHeadController : public EyeHeadController
{
public:
  //! Constructor. See the base classes for parameters.
  SimpleEyeHeadController(OptionManager& mgr);

  //! Destructor
  virtual ~SimpleEyeHeadController();

protected:
  //! Callback for every clock tick
  SIMCALLBACK_DECLARE(SimpleEyeHeadController, SimEventClockTick);

  nub::ref<SaccadeControllerEyeConfigurator> itsSCCeye;   //!< eye SCC
  nub::ref<SaccadeControllerHeadConfigurator> itsSCChead; //!< head SCC

  nub::soft_ref<SaccadeController> itsSCeye; //!< shortcut
  nub::soft_ref<SaccadeController> itsSChead; //!< shortcut

  virtual void start1(); //!< get started
};

// ######################################################################
//! EyeTracker eye/head controller
/*! This conroller contains one or more EyeTrackerSaccadeController
  objects. It can hence be used to run a simulation in conjunction
  with some already-available human/monkey/other eye-tracking
  traces. Typically this would be used in conjunction with
  SimulationViewerEyeMvt. */
class EyeTrackerEyeHeadController : public EyeHeadController
{
public:
  //! Constructor. See the base classes for parameters.
  EyeTrackerEyeHeadController(OptionManager& mgr);

  //! Destructor
  virtual ~EyeTrackerEyeHeadController();

protected:
  //! Callback for every clock tick
  SIMCALLBACK_DECLARE(EyeTrackerEyeHeadController, SimEventClockTick);

  virtual void start1(); //!< get started
  OModelParam<std::string> itsConfig; //!< our config string
  std::vector<rutz::shared_ptr<EyeTrace> > itsEyeTrace; //!< our eye traces
  std::vector<size_t> itsEyeSample; //!< eye sample number for each tracker
};


// ######################################################################
//! Realistic rhesus monkey eye/head controller
/*! This is a hack based on Nitin's old MonkeySaccadeController but
  decoupling eye and head (so that the head minds its own business
  without influencing the eye). Not as realistic in the details, but
  the end result is the same. In addition to having plug-in eye and
  head saccade controllers, we here have an additional built-in
  ThresholdSaccadeController which is used to detect when the eye is
  stable.  */
class MonkeyEyeHeadController : public EyeHeadController
{
public:
  //! Constructor
  MonkeyEyeHeadController(OptionManager& mgr);

  //! Destructor
  virtual ~MonkeyEyeHeadController();

  //! Receive a new percept (e.g., winner from saliency map)
  virtual void setPercept(const WTAwinner& fix, SimEventQueue& q);

protected:
  //! Callback for every clock tick
  SIMCALLBACK_DECLARE(MonkeyEyeHeadController, SimEventClockTick);

  OModelParam<SimTime> itsBlinkWait; //!< wait time before a blink
  OModelParam<SimTime> itsBlinkDur;  //!< blink duration
  OModelParam<float> itsOdist;

private:
  nub::ref<SpatialMetrics> itsMetrics; //!< metrics that depend on input size
  nub::ref<ThresholdSaccadeController> itsTSC;
  SimTime lastsbt; // time of end of last saccade/blink
  SimTime blinkt;  // time of start of current blink

  // compute amplitude of head displacement. All angles in degrees
  double headAmplitude(const double curreye, const double currhead,
                       const double target);
};



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_EYEHEADCONTROLLERS_H_DEFINED
