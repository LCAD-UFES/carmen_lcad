/*!@file Neuro/HandControllers.H Hand controllers */

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
// Primary maintainer for this file: Dicky Nauli Sihite <sihite@usc.edu>
// $HeadURL:
// $Id:
//

#ifndef NEURO_HANDCONTROLLERS_H_DEFINED
#define NEURO_HANDCONTROLLERS_H_DEFINED

#include "Neuro/HandController.H"
#include "Neuro/SaccadeControllers.H"
#include "Neuro/SaccadeControllerConfigurator.H"
#include "Simulation/SimEvents.H"

#include <vector>

class HandTrace;

// ######################################################################
/*!>Stub hand controller
 *  No-op implementation -- never makes a decision, no matter what hand
 *  SaccadeControllers it may hold.
 */

class StubHandController : public HandController
{
public:
  //! Constructor
  StubHandController(OptionManager& mgr);

  //! Destructor
  virtual ~StubHandController();
};

// ######################################################################
/*!>HandTracker hand controller
 *  This controller contains one or more TrackerHandController
 *  objects. It can hence be used to run a simulation in conjunction
 *  with some already-available human/monkey/other eye-tracking
 *  traces. Typically this would be used in conjunction with
 *  SimulationViewerHand. */
class TrackerHandController : public HandController
{
public:
  //! Constructor. See the base classes for parameters.
  TrackerHandController(OptionManager& mgr);

  //! Destructor
  virtual ~TrackerHandController();

protected:
  //! Callback for every clock tick
  SIMCALLBACK_DECLARE(TrackerHandController, SimEventClockTick);

  virtual void start1(); //!< get started
  OModelParam<std::string> itsConfig; //!< our config string
  std::vector<rutz::shared_ptr<HandTrace> > itsHandTrace; //!< our hand traces
  std::vector<size_t> itsHandSample; //!< hand sample number for each tracker
};



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_HANDCONTROLLERS_H_DEFINED
