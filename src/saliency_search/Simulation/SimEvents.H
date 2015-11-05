/*!@file Simulation/SimEvents.H Instantiations of SimEvent */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Simulation/SimEvents.H $
// $Id: SimEvents.H 10709 2009-02-01 02:11:15Z itti $
//

#ifndef SIMULATION_SIMEVENTS_H_DEFINED
#define SIMULATION_SIMEVENTS_H_DEFINED

#include "Simulation/SimEvent.H"
#include "Util/SimTime.H"

// Only declare very generic SimEvent derivatives here (basically,
// which may be used by many modules and depend only on very low-level
// things). For more specialized SimEvent derivatives, declare them in
// corresponding files in the appropriate directories. For example,
// Neuro/NeuroSimEvents.H

// ######################################################################
//! Request to break the simulation
/*! Post this and the next SimEventQueue::evolve() we will return with
  a break status, usually resulting in terminating the simulation. */
class SimEventBreak : public SimEvent {
public:
  //! Constructor
  SimEventBreak(SimModule* src, const std::string& reason = "");

  //! Destructor
  virtual ~SimEventBreak();

  //! Explain why we break
  virtual std::string toString() const;

private:
  std::string itsReason;
};

// ######################################################################
//! Wait until user presses RETURN key
/*! A SimModule may post this to request that the simulation be paused
  until the user presses the RETURN key. For example, this may be used
  after a bunch of images have been displayed, to give the user the
  time to examine them before continuing with the simulation. This
  event is handled directly at the very beginning of
  SimEventQueue::evolve(), just before we increment the clock. The
  event is displayed, plus a message to invite the user to press
  RETURN to continue. */
class SimEventUserWait : public SimEvent {
public:
  //! Constuctor
  SimEventUserWait(SimModule* src, const std::string& reason = "");

  //! Destructor
  virtual ~SimEventUserWait();

  //! Explain why we pause
  virtual std::string toString() const;

private:
  std::string itsReason;
};

// ######################################################################
//! Requests that memory stats be shown now
/*! A SimModule may post this to request that statistics about memory
  usage be displayed now. This requires that the user specified
  --mem-stats on the command-line, otherwise this event will be
  ignored. A frame number is optional, as well as an allocation unit
  to show the stats in. */
class SimEventShowMemStats : public SimEvent {
public:
  //! Constuctor
  SimEventShowMemStats(SimModule* src,
                       const int fram = -1, const size_t uni = 0);

  //! Destructor
  virtual ~SimEventShowMemStats();

  //! Get frame
  int frame() const;

  //! Get allocation unit
  size_t unit() const;

private:
  int itsFrame;
  int itsUnit;
};

// ######################################################################
//! Save your outputs
/*! SimOutputFrameSeries (or others) may post this when it is time to
  save the next round of outputs. SimModule objects may want to catch
  this if they have possible outputs to save. They should then use the
  ModelComponentSaveInfo data contained here in the event to decide
  how and where to save their stuff. */
class SimEventSaveOutput : public SimEvent {
public:
  //! Constuctor
  SimEventSaveOutput(SimModule* src,
                     const rutz::shared_ptr<ModelComponentSaveInfo>& sinfo);

  //! Destructor
  virtual ~SimEventSaveOutput();

  //! Get the saveinfo data
  const ModelComponentSaveInfo& sinfo() const;

private:
  const rutz::shared_ptr<ModelComponentSaveInfo> itsSinfo;
};

// ######################################################################
//! Posted by SimEventQueue at the beginning of each clock tick
/*! A SimModule that registers a callback for this event type will
    have its callback triggered at every clock tick. */
class SimEventClockTick : public SimEvent {
public:
  //! Constuctor
  SimEventClockTick(SimModule* src, const SimTime& t);

  //! Destructor
  virtual ~SimEventClockTick();

  //! Time at which the event was created
  const SimTime& time() const;

  //! Show the time
  virtual std::string toString() const;

private:
  const SimTime itsTime;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // SIMULATION_SIMEVENTS_H_DEFINED
