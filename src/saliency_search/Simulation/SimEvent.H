/*!@file Simulation/SimEvent.H Base class for events that arise in simulation*/

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Simulation/SimEvent.H $
// $Id: SimEvent.H 13103 2010-03-31 02:24:47Z itti $
//

#ifndef SIMULATION_SIMEVENT_H_DEFINED
#define SIMULATION_SIMEVENT_H_DEFINED

#include "rutz/shared_ptr.h"
#include "Simulation/SimModule.H"
#include <set>

//! Base class for events that arise in simulation
/*! Actual events would normally derive from this base. See
  Simulation/SimEvents.H */
class SimEvent {
public:
  //! Constuctor
  /*! Normally a SimModule would pass "this" as src. This is used in SimEventQueue to prune older events that have been
    pushed by the same SimModule. The event will be visible to anyone through SimEventQueue::check() after it has been
    posted to the SimEventQueue. */
  SimEvent(SimModule* src);

  //! Destructor
  virtual ~SimEvent();

  //! Get a description for printing out
  /*! The base class implementation only returns the demangled name of the class. Overload as necessary in derived
    classes if you want to also return additional information. */
  virtual std::string toString() const;

  //! Get comma-separated list of receivers done with the event
  /*! Normally, few people except for SimEventQueue would use this function. */
  std::string getDoneList() const;

private:
  friend class SimEventQueue; // allow SimEventQueue to access our private data

  //! Returns true if receiver is done with event
  /*! Normally, few people except for SimEventQueue would use this function. */
  bool isDone(const SimModule* const receiver) const;

  //! Mark that receiver is done with the event
  /*! Normally, few people except for SimEventQueue would use this function. */
  void markDone(const SimModule* const receiver);

  SimModule* const itsSource; // source of the event (a brain module)
  std::set<const SimModule*> itsReceiversDone;
};


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // SIMULATION_SIMEVENT_H_DEFINED
