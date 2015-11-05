/*!@file Simulation/SimEvents.C  Instantiations of SimEvent */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Simulation/SimEvents.C $
// $Id: SimEvents.C 10656 2009-01-22 22:57:23Z itti $
//

#include "Simulation/SimEvents.H"
#include "Util/sformat.H"

// ######################################################################
SimEventBreak::SimEventBreak(SimModule* src, const std::string& reason) :
  SimEvent(src), itsReason(reason)
{ }

SimEventBreak::~SimEventBreak()
{ }

std::string SimEventBreak::toString() const
{
  if (itsReason.empty())
    return SimEvent::toString();
  else
    return SimEvent::toString() + sformat(" [%s]", itsReason.c_str());
}

// ######################################################################
SimEventUserWait::SimEventUserWait(SimModule* src,
                                   const std::string& reason) :
  SimEvent(src), itsReason(reason)
{ }

SimEventUserWait::~SimEventUserWait()
{ }

std::string SimEventUserWait::toString() const
{
  if (itsReason.empty())
    return SimEvent::toString();
  else
    return SimEvent::toString() + sformat(" [%s]", itsReason.c_str());
}

// ######################################################################
SimEventShowMemStats::SimEventShowMemStats(SimModule* src,
                                           const int fram,
                                           const size_t uni) :
  SimEvent(src), itsFrame(fram), itsUnit(uni)
{ }

SimEventShowMemStats::~SimEventShowMemStats()
{ }

int SimEventShowMemStats::frame() const
{ return itsFrame; }

size_t SimEventShowMemStats::unit() const
{ return itsUnit; }

// ######################################################################
SimEventSaveOutput::
SimEventSaveOutput(SimModule* src,
                   const rutz::shared_ptr<ModelComponentSaveInfo>& sinfo) :
  SimEvent(src), itsSinfo(sinfo)
{ }

SimEventSaveOutput::~SimEventSaveOutput()
{ }

const ModelComponentSaveInfo& SimEventSaveOutput::sinfo() const
{ return *itsSinfo; }

// ######################################################################
SimEventClockTick::SimEventClockTick(SimModule* src, const SimTime& t) :
  SimEvent(src), itsTime(t)
{ }

SimEventClockTick::~SimEventClockTick()
{ }

const SimTime& SimEventClockTick::time() const
{ return itsTime; }

std::string SimEventClockTick::toString() const
{
  return SimEvent::toString() + sformat(" [%.2fms]", itsTime.msecs());
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
