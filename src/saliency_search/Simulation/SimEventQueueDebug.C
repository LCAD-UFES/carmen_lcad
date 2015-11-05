/*!@file Simulation/SimEventQueueDebug.C Dispatch simulation events */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Simulation/SimEventQueueDebug.C $
// $Id: SimEventQueueDebug.C 11082 2009-04-02 23:31:50Z itti $
//

#include "Simulation/SimEventQueueDebug.H"

#include "Simulation/SimEvents.H"  // for SimEventBreak
#include "rutz/demangle.h"

// ######################################################################
SimEventQueueDebug::SimEventQueueDebug(OptionManager& mgr,
                                       const std::string& descrName,
                                       const std::string& tagName,
                                       const SimTime starttime) :
  SimEventQueue(mgr, descrName, tagName, starttime)
{ }

// ######################################################################
SimEventQueueDebug::~SimEventQueueDebug()
{ }

// ######################################################################
SimStatus SimEventQueueDebug::evolve()
{
  LINFO("*** Queue contents before evolve() at t=%fms:", t.msecs());

  SeqData::const_iterator itr = itsQueue.begin(), stop = itsQueue.end();
  while (itr != stop) {
    const char *rname = itr->first.c_str();
    const RealmData *rd = &(itr->second);
    const SeqEntryVec *seq = &(rd->events);

    if (seq->size()) {
        SeqEntryVec::const_iterator e = seq->begin(), stop = seq->end();

        while (e != stop) {
            LINFO("*** [%s] %s posted at t=%fms [done by: %s]",
                  rname, e->second->toString().c_str(),
                  e->first.msecs(), e->second->getDoneList().c_str());
            ++e;
        }
    } else LINFO("*** [Queue is empty]");

    ++itr;
  }
  return SimEventQueue::evolve();
}

// ######################################################################
void SimEventQueueDebug::clear()
{
  LINFO("Clearing the event queue...");
  SimEventQueue::clear();
}

// ######################################################################
void SimEventQueueDebug::resetTime(const SimTime& tim)
{
  LINFO("Resetting time to %fms and clearing queue...", tim.msecs());
  SimEventQueue::resetTime(tim);
}

// ######################################################################
void SimEventQueueDebug::prune(const SimTime& tt)
{
  LINFO("Pruning events with t < %fms", tt.msecs());
  SimEventQueue::prune(tt);
}

// ######################################################################
void SimEventQueueDebug::postHelper(const std::type_info& etype,
                                    const rutz::shared_ptr<SimEvent>& e)
{
  LINFO("%s, t=%fms", e->toString().c_str(), t.msecs());
}


// ######################################################################
void
SimEventQueueDebug::checkHelper(const std::type_info& etype,
                                const rutz::shared_ptr<SimEvent>& e,
                                const SimModule* caller,
                                const SimEventQueueFlag flags,
                                const SimModule* eventsrc)
{
  if (eventsrc)
    LINFO("%s:%s: Returning [%s] (request narrowed down to %s)",
          caller ? caller->descriptiveName().c_str() :
          "[Anonymous Caller]", rutz::demangled_name(etype),
          e.is_valid() ? e->toString().c_str() : "No match",
          eventsrc->descriptiveName().c_str());
  else
    LINFO("%s:%s: Returning [%s]",
          caller ? caller->descriptiveName().c_str() :
          "[Anonymous Caller]", rutz::demangled_name(etype),
          e.is_valid() ? e->toString().c_str() : "No match");

}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
