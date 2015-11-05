/*!@file Simulation/SimEventQueueDebug.H Dispatch simulation events */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Simulation/SimEventQueueDebug.H $
// $Id: SimEventQueueDebug.H 11082 2009-04-02 23:31:50Z itti $
//

#ifndef SIMULATION_SIMEVENTQUEUEDEBUG_H_DEFINED
#define SIMULATION_SIMEVENTQUEUEDEBUG_H_DEFINED

#include "Simulation/SimEventQueue.H"

//! Debug version of SimEventQueue
/*! This version just posts a number of messages about the queue when
  events are posted, retrieved, when the queue evolves, etc. */
class SimEventQueueDebug : public SimEventQueue {
public:
  //! Constructor
  SimEventQueueDebug(OptionManager& mgr,
                const std::string& descrName = "Simulation Event Queue Debug",
                const std::string& tagName = "SimEventQueue",
                const SimTime starttime = SimTime::ZERO());

  //! Destructor
  virtual ~SimEventQueueDebug();

  //! Increment the master clock and delete old events
  /*! In addition to the normal behavior, we dump the queue. */
  virtual SimStatus evolve();

  //! Erase the entire queue
  /*! In addition to the normal behavior, we report. */
  virtual void clear();

  //! Erase all events with time <= tt
  /*! In addition to the normal behavior, we report. */
  virtual void prune(const SimTime& tt);

  //! Reset the time to the given value and erase the entire queue
  /*! In addition to the normal behavior, we report. */
  virtual void resetTime(const SimTime& tim);

protected:
  //! This is called by post()
  virtual void postHelper(const std::type_info& etype,
                          const rutz::shared_ptr<SimEvent>& e);

  //! this is called by check()
  virtual void checkHelper(const std::type_info& etype,
                           const rutz::shared_ptr<SimEvent>& e,
                           const SimModule* caller,
                           const SimEventQueueFlag flags,
                           const SimModule* eventsrc);
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // SIMULATION_SIMEVENTQUEUE_H_DEFINED
