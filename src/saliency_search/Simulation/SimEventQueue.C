/*!@file Simulation/SimEventQueue.C Dispatch simulation events */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Simulation/SimEventQueue.C $
// $Id: SimEventQueue.C 12962 2010-03-06 02:13:53Z irock $
//

#include "Simulation/SimEventQueue.H"

#include "Component/ModelOptionDef.H"
#include "Component/GlobalOpts.H"
#include "Simulation/SimEvents.H"
#include "Simulation/SimulationOpts.H"
#include "Util/AllocAux.H"
#include "Util/sformat.H"
#include "Util/TextLog.H"

#include <cstdio>

static const ModelOptionDef OPT_ShowMemStats =
  { MODOPT_FLAG, "ShowMemStats", &MOC_GENERAL, OPTEXP_CORE,
    "Show verbose memory allocation statistics.",
    "mem-stats", '\0', "", "false" };

static const ModelOptionDef OPT_ShowMemStatsUnits =
  { MODOPT_ARG(size_t), "ShowMemStatsUnits", &MOC_GENERAL, OPTEXP_CORE,
    "Allocation unit size (in bytes) to use when displaying the "
    "verbose memory statistics, or 0 to let simulation modules decide "
    "on an allocation unit each time they request that stats be shown.",
    "mem-stats-units", '\0', "<bytes>", "0" };

// ######################################################################
SimEventQueue::SimEventQueue(OptionManager& mgr,
                             const std::string& descrName,
                             const std::string& tagName,
                             const SimTime starttime) :
  SimModule(mgr, descrName, tagName),
  itsTimeStep(&OPT_SimulationTimeStep, this),
  itsTooMuchTime(&OPT_SimulationTooMuchTime, this),
  itsShowMemStats(&OPT_ShowMemStats, this),
  itsShowMemStatsUnits(&OPT_ShowMemStatsUnits, this),
  itsLogFile(&OPT_TextLogFile, this),
  t(starttime), itsQueue()
{ }

// ######################################################################
SimEventQueue::~SimEventQueue()
{ }

// ######################################################################
void SimEventQueue::start1()
{
  SimModule::start1();

  // make sure the realm in which we are exists, since we will be
  // posting events into it (e.g., SimEventClockTick), even if nobody
  // is going to catch them and hence that realm might not have been
  // created yet:
  getRealm(realm(), true);
}

// ######################################################################
void SimEventQueue::evolve(SimEventQueue& q)
{
  LFATAL("Never call this function!");
}

// ######################################################################
SimStatus SimEventQueue::evolve()
{
  // our clock is ticking! Let everyone know:
  rutz::shared_ptr<SimEventClockTick> ee(new SimEventClockTick(this, now()));
  post(ee);

  // did anyone request that we show memory stats?
  if (itsShowMemStats.getVal())
    {
      if (SeC<SimEventShowMemStats> e = check<SimEventShowMemStats>(this))
        {
          LINFO("##### %s #####", e->toString().c_str());

          // gobble up all other requests. NOTE: if several requests had
          // different units we only show the first one. Maybe a FIXME:
          while (SeC<SimEventShowMemStats> ee = check<SimEventShowMemStats>(this))
            LINFO("##### %s #####", ee->toString().c_str());

          showMemStats(e->frame(), e->unit());
        }
    }

  // did anyone post a userwait event?
  if (SeC<SimEventUserWait> e = check<SimEventUserWait>(this))
    {
      // make sure we only pause once even though several modules may
      // have requested a pause:
      do {
        LINFO("##### %s #####", e->toString().c_str());
        e = check<SimEventUserWait>(this);
      } while(e);

      printf("<<<<< Press [RETURN] to continue >>>>>\n");
      char tmp[10];
      char* ret = fgets(tmp, 10, stdin);
      if (ret);
    }

  // did anyone post a break event?
  if (SeC<SimEventBreak> e = check<SimEventBreak>(this))
    {
      // make sure we report all break requests:
      do {
        LINFO("##### %s #####", e->toString().c_str());
        e = check<SimEventBreak>(this);
      } while(e);

      LINFO("##### Break requested -- DONE #####");
      showMemStats(-2, 0);
      return SIM_BREAK;
    }

  // have we run for too long?
  if (t > itsTooMuchTime.getVal())
    {
      LINFO("#### Too much time elapsed -- DONE ####");
      showMemStats(-2, 0);
      return SIM_BREAK;
    }

  // increment t by one step:
  t += itsTimeStep.getVal();

  // let our TextLog know about the new time:
  setLogTime(itsLogFile.getVal(), t);

  // keep running...
  return SIM_CONTINUE;
}

// ######################################################################
void SimEventQueue::clear()
{
  SeqData::iterator itr = itsQueue.begin(), stop = itsQueue.end();
  while (itr != stop) {
    RealmData *rd = &(itr->second);
    SeqEntryVec *seq = &(rd->events);
    seq->clear();
    ++itr;
  }
}

// ######################################################################
void SimEventQueue::prune(const SimTime& tt)
{
  SeqData::iterator itr = itsQueue.begin(), stop = itsQueue.end();
  while (itr != stop) {
    RealmData *rd = &(itr->second);
    SeqEntryVec *seq = &(rd->events);

    // delete all events that are <= tt:
    SeqEntryVec::iterator ee = seq->begin();
    while (ee != seq->end()) if (ee->first <= tt) ee = seq->erase(ee); else ++ee;

    ++itr;
  }
}

// ######################################################################
const SimTime& SimEventQueue::now() const
{ return t; }

// ######################################################################
void SimEventQueue::resetTime(const SimTime& tim)
{
  this->clear();
  t = tim;
}
// ######################################################################
SimEventQueue::RealmData* SimEventQueue::getRealm(const std::string& realmname, const bool create_on_fail)
{
  // do we already know about thsi realm?
  SeqData::iterator itr = itsQueue.find(realmname);
  if (itr == itsQueue.end())
    {
      if (create_on_fail)
        {
          // first time we encounter this realm, let's create an entry for it:
          RealmData rd;
          std::pair<SeqData::iterator, bool> ii = itsQueue.insert(SeqData::value_type(realmname, rd));
          if (ii.second == false) LFATAL("Error creating realm [%s]", realmname.c_str());
          itr = ii.first; // use that new entry in the rest of this function
        }
      else
         LFATAL("Unknown realm [%s]", realmname.c_str());
    }

  return &(itr->second);
}

// ######################################################################
void SimEventQueue::registerSimCallbackClient(SimModule *s)
{
  // get the realm, create it if new:
  SimEventQueue::RealmData *rd = getRealm(s->realm(), true);

  // get the callback map for this realm:
  scbm *themap = &(rd->callbacks);

  // loop over all of the client's callbacks and register them:
  typedef std::vector<SimCallbackBase *> vscbb;
  vscbb *v = &(s->itsSimCallbacks);
  vscbb::const_iterator c = v->begin(), stop = v->end();
  while (c != stop) {
    SimCallbackBase *cb = (*c);

    // get the event type:
    const std::type_info *ti = &cb->etype();

    // do we already have registered callbacks for this event type?
    scbm::iterator i = themap->find(ti);

    if (i == themap->end()) {
      // first time we encounter this event type, create a new map entry for it:
      rutz::shared_ptr<sscbb> newset(new sscbb());
      std::pair<scbm::iterator, bool> ii = themap->insert(scbm::value_type(ti, newset));
      if (ii.second == false)
        LFATAL("Error registering callback [%s] in realm [%s]", cb->toString().c_str(), s->realm().c_str());
      i = ii.first;
    }

    // add the callback to our set of callbacks for that event type:
    i->second->insert(cb);

    LDEBUG("Registered [%s] in realm [%s]", cb->toString().c_str(), s->realm().c_str());

    ++c;
  }
}

// ######################################################################
void SimEventQueue::registerSimReqHandlerClient(SimModule *s)
{
  // get the realm, create it if new:
  SimEventQueue::RealmData *rd = getRealm(s->realm(), true);

  // get the handlers map for this realm:
  srhm *themap = &(rd->reqhandlers);

  // loop over all of the client's request handlers and register them:
  typedef std::vector<SimReqHandlerBase *> vsrhb;
  vsrhb *v = &(s->itsSimReqHandlers);
  vsrhb::const_iterator c = v->begin(), stop = v->end();
  while (c != stop) {
    SimReqHandlerBase *cb = (*c);

    // get the req type:
    const std::type_info *ti = &cb->rtype();

    // do we already have registered callbacks for this event type?
    srhm::iterator i = themap->find(ti);

    if (i == themap->end()) {
      // first time we encounter this req type, create a new map entry for it:
      rutz::shared_ptr<ssrhb> newset(new ssrhb());
      std::pair<srhm::iterator, bool> ii = themap->insert(srhm::value_type(ti, newset));
      if (ii.second == false)
        LFATAL("Error registering handler [%s] in realm [%s]", cb->toString().c_str(), s->realm().c_str());
      i = ii.first;
    }

    // add the handler to our set of handlers for that req type:
    i->second->push_back(cb);

    LDEBUG("Registered [%s] in realm [%s]", cb->toString().c_str(), s->realm().c_str());

    ++c;
  }
}

// ######################################################################
void SimEventQueue::printCallbacks() const
{
  SeqData::const_iterator itr = itsQueue.begin(), stop = itsQueue.end();
  while (itr != stop) {
    const char *rname = itr->first.c_str();
    const RealmData *rd = &(itr->second);
    const scbm *cmap = &(rd->callbacks);
    const srhm *rmap = &(rd->reqhandlers);

    scbm::const_iterator c = cmap->begin(), sto = cmap->end();
    while (c != sto) {
      rutz::shared_ptr<sscbb> s = c->second;
      sscbb::const_iterator i = s->begin(), fini = s->end();
      while (i != fini) LINFO("[%s] %s", rname, (*i++)->toString().c_str());
      ++c;
    }
    srhm::const_iterator cc = rmap->begin(), sstop = rmap->end();
    while (cc != sstop) {
      rutz::shared_ptr<ssrhb> s = cc->second;
      ssrhb::const_iterator i = s->begin(), fini = s->end();
      while (i != fini) LINFO("[%s] %s", rname, (*i++)->toString().c_str());
      ++cc;
    }
    ++itr;
  }
}

// ######################################################################
void SimEventQueue::reset1()
{
  resetTime();

  // propagate to our base class:
  SimModule::reset1();
}

// ######################################################################
void SimEventQueue::showMemStats(const int frame, const size_t units) const
{
  size_t u = itsShowMemStatsUnits.getVal();
  if (u == 0)
    {
      if (units != 0) u = units;
      // else ... just leave u=0 so that invt_allocation_show_stats()
      // can fall back to whatever default value has been passed to
      // invt_allocation_set_stats_units()
    }

  if (frame >= 0)
    {
      SHOWMEMORY("MEMORY USAGE: frame %d t=%.1fms", frame, t.msecs());
      invt_allocation_show_stats
        (1, sformat("frame %06d", frame).c_str(), u);
    }
  else if (frame == -1)
    {
      SHOWMEMORY("MEMORY USAGE: t=%.1fms", t.msecs());
      invt_allocation_show_stats(1, "", u);
    }
  else
    {
      SHOWMEMORY("FINAL MEMORY USAGE: t=%.1fms", t.msecs());
      invt_allocation_show_stats(1, "final", u);
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
