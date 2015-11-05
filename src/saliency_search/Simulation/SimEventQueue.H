/*!@file Simulation/SimEventQueue.H Dispatch simulation events */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Simulation/SimEventQueue.H $
// $Id: SimEventQueue.H 13103 2010-03-31 02:24:47Z itti $
//

#ifndef SIMULATION_SIMEVENTQUEUE_H_DEFINED
#define SIMULATION_SIMEVENTQUEUE_H_DEFINED

#include "Component/ModelParam.H"
#include "Simulation/SimEvent.H"
#include "Simulation/SimModule.H"
#include "Simulation/SimCallback.H"
#include "Simulation/SimCallbackClient.H"
#include "Simulation/SimReq.H"
#include "Simulation/SimReqHandler.H"
#include "Simulation/SimReqHandlerClient.H"
#include "Simulation/SimEventQueueFlag.H"
#include "Util/SimTime.H"
#include "rutz/demangle.h"
#include <vector>
#include <map>
#include <set>

//! Simple enum to return a status after evolve()
enum SimStatus
  {
    SIM_CONTINUE,
    SIM_BREAK
  };

//! Helper class to check for events (defined below)
template <class T> class SeC;

//! Helper class to gather events generated while running a simulation
/*! The class maintains a list of events which may arise from various
  simulation modules, and can be queried for specific events.

  This class implements a so-called 'blackboard architecture', a
  concept which was introduced in artificial intelligence research to
  facilitate information sharing among multiple heterogeneous
  problem-solving agents. The basic idea is that the blackboard is a
  shared repository of information which all agents can access while
  working towards the solution to a given problem.

  Because different modules are usually run sequentially, e.g., in
  Brain::evolve(), the intent here is that we keep track of both the
  events that arise during a given evolve at SimTime t and during the
  previous evolve. If an event arises after a given module has already
  been evolve'd for the current iteration, that module will know about
  the event on the next iteration. For example, in Brain::evolve(), we
  evolve the SaliencyMap module before the EyeHeadController module;
  hence if an eye saccade is initiated by the EyeHeadController, the
  SaliencyMap will become aware of that at the next iteration, and may
  decide to take appropriate action, e.g., saccadic suppression. To
  eliminate events older than a given time, use prune(), typically
  this would be done at the beginning of Brain::evolve().

  IMPORTANT: When a given SimModule posts an event at time t, all
  events posted by that same SimModule at times < t are erased from
  the queue. This is as soon as the event type matches, whether or not
  the event's contents match.

  HISTORICAL NOTE: SimEventQueue was introduced as an attempt to give
  more autonomy to the Brain modules by employing a blackboard
  architecture. An example of how things worked before is as follows:
  a SaccadeController may decide in its evolve() function that it is
  time to blink, and will switch itself to blink state.
  Brain::evolve() would have to query the controller after running
  evolve() on it, to check whether anything happened, like for example
  the start of the blink. If so, Brain::eyeBlink() would be triggered,
  which in turn would dispatch the event to several other modules,
  calling for example SimulationViewer::eyeBlink() and
  SaliencyMap::blinkSuppression(). The more types of events could
  occur, the more complex Brain::evolve() was becoming. With
  SimEventQueue, most of the complex orchestration which was in
  Brain::evolve() is now delegated to the brain modules
  themselves. For example, when SaccadeController initiates a blink,
  it simply posts a SimEvent indicating so. When SimulationViewer's
  turn comes to evolve, it can query SimEventQueue to check whether a
  blink has started, and take appropriate action. */
class SimEventQueue : public SimModule {
public:
  //! Constructor
  /*! An optional start time for the master simulation clock may be
    provided. */
  SimEventQueue(OptionManager& mgr,
                const std::string& descrName = "Simulation Event Queue",
                const std::string& tagName = "SimEventQueue",
                const SimTime starttime = SimTime::ZERO());

  //! Destructor
  virtual ~SimEventQueue();

  //! Post a new SimEvent; typically, SimModule modules do this
  template <class E> inline
  void post(rutz::shared_ptr<E> e);

  //! Handle a new SimReq; typically, SimModule modules do this
  /*! Note two differences between this and post(): here, the SimReq
      object does not end up in the queue, it is just privately passed
      to the event handlewr, who will modify it, and then the caller
      can use the results. Second, SimReq objects are typically passed
      read/write while SimEvent objects are read-only in post(). This
      function returns the number of handlers that were triggered by
      the request. Zero means that nobody caught the request. Default
      behavior is to throw a fatal exception if zero or more than 1
      handlers catch the request. */
  template <class R> inline
  size_t request(rutz::shared_ptr<R> r,
               const SimReqQueueFlag flags = SRQ_NONE_FATAL | SRQ_SEVERAL_FATAL);

  //! Return event, if any, that satisfies the given criteria
  /*! Example usage is as follows:
    \code

    // Assume a SimEventQueue object q
    // Assume that caller is a SimModule
    // Assume that CovertShift derives from SimEvent
    if (SeC<CovertShift> e = q.check<CovertShift>(this))
      {
        // handle the event; you can use 'e' pretty much as you
        // would use a shared_ptr
      }

    \endcode

    You can specify an eventsrc SimModule to further narrow the search
    to only events that originated from this module. NOTE: only events
    which are either public or intended for the caller will be
    returned (assuming that all other match criteria are also
    fullfilled). See Simulation/SimEvent.H for the distinction between
    public and private events. By default, the flags are set so that
    we will return the most recent matching unmarked event, then mark
    it in the queue as done by the caller module; hence several calls
    to check will return all unmarked events (from most recent to
    oldest) until they have all been marked.  PROGRAMMER NOTE: don't
    overload this, instead you can overload checkHelper() which is
    called from here. */
  template <class E> inline
  SeC<E> check(const SimModule* caller,
               const SimEventQueueFlag flags = SEQ_UNMARKED | SEQ_MARK,
               const SimModule* eventsrc = 0);

  //! Increment the master clock and delete old events
  /*! Events whose time is < t just before we increment the clock will
    be removed from the queue. We will here also check for some events
    that are handled directly by the queue agent. */
  virtual SimStatus evolve();

  //! Get the master clock's time
  virtual const SimTime& now() const;

  //! Erase the entire queue; like reset() but does not reset time
  /*! Use reset() instead unless you know what you are doing. */
  virtual void clear();

  //! Erase all events with time <= tt
  virtual void prune(const SimTime& tt);

  //! Reset the time to the given value and erase the entire queue
  /*! Like reset() but does not propagate to subcomponents.  Use
   reset() instead unless you know what you are doing. */
  virtual void resetTime(const SimTime& tim = SimTime::ZERO());

  //! Register a new SimCallbackClient, taking all its callbacks
  void registerSimCallbackClient(SimModule *s);

  //! Register a new SimReqHandlerClient, taking all its callbacks
  void registerSimReqHandlerClient(SimModule *s);

  //! Show all registered callbacks
  void printCallbacks() const;

protected:
  //! Simulation time step, in seconds
  OModelParam<SimTime> itsTimeStep;

  //! Exit after a given time in seconds (if not zero)
  OModelParam<SimTime> itsTooMuchTime;

  //! Display memory stats when a module posts a SimEventShowMemStats
  OModelParam<bool> itsShowMemStats;

  //! Default units for memory stats
  OModelParam<size_t> itsShowMemStatsUnits;

  //! text log file name
  OModelParam<std::string> itsLogFile;

  SimTime t;  //!< the simulation's master clock

  //! This is called by post(). Default is a no-op.
  /*! @param etype typeid of the event which the caller
    wanted to check for (use rutz::demangle(etype) to get the type name)
    @param e the event posted
    @param the post time */
  inline virtual
  void postHelper(const std::type_info& etype,
                  const rutz::shared_ptr<SimEvent>& e);

  //! This is called by request(). Default is a no-op.
  inline virtual
  void requestHelper(const std::type_info& etype,
                     const rutz::shared_ptr<SimReq>& e);

  //! this is called by check(). Default is a no-op.
  /*! @param etype typeid of the event which the caller
    wanted to check for (use rutz::demangle(etype) to get the type name)
    @param e matching event found, will be invalid if no match was
    found. Note that e may have a different type from what the caller
    was looking for, namely it could be a derived class from the one
    looked for.
    @param caller the SimModule that called check().
    @param flags the flags passed to check().
    @param eventsrc the event source passed to check() if any.  */
  inline virtual
  void checkHelper(const std::type_info& etype,
                   const rutz::shared_ptr<SimEvent>& e,
                   const SimModule* caller,
                   const SimEventQueueFlag flags,
                   const SimModule* eventsrc);

  //! Reset and propagate the reset signal to the sub-components
  /*! See the base function in ModelComponent.H for info. */
  virtual void reset1();

  //! get started
  virtual void start1();

  struct TypeInfoCompare {
    bool operator()(const std::type_info* t1, const std::type_info* t2) const
    { return (t1->before(*t2)); }
  };

  // our queue of SimEvents is segmented into realms by using a map
  // with the realm name as key; for each realm, we have events,
  // callbacks, and request handlers:

  // Data for events: within each realm, we simply pile them up into a
  // vector and keep track of their time:
  typedef std::pair<SimTime, rutz::shared_ptr<SimEvent> > SeqEntry;
  typedef std::vector<SeqEntry> SeqEntryVec;

  // Data for callback management:
  // sscbb is a multiset type where we just pile up all the callbacks
  //   for a given event type (in the given realm), using some ordering by
  //   priority and insertion order.
  // scbm is a map of sscbb where each entry corresponds to a given event
  //   type (in the current realm)
  typedef std::multiset<SimCallbackBase*, SimCallbackCompare> sscbb;
  typedef std::map<const std::type_info*, rutz::shared_ptr<sscbb>, TypeInfoCompare> scbm;

  // Data for SimReq handler management:
  typedef std::vector<SimReqHandlerBase*> ssrhb;
  typedef std::map<const std::type_info*, rutz::shared_ptr<ssrhb>, TypeInfoCompare> srhm;

  // we have one of these for each realm:
  typedef struct {
    SeqEntryVec events;
    scbm callbacks;
    srhm reqhandlers;
  } RealmData;

  typedef std::map<std::string, RealmData> SeqData;
  SeqData itsQueue;

  // get access to the RealmData for a given realm; if create_on_fail
  // is true, a new realm will be created if one was not found by that
  // name; otherwise a fatal error will occur if realm not found:
  RealmData* getRealm(const std::string& realmname, const bool create_on_fail);

private:
  // prevent people from calling SimModule::evolve() on us:
  virtual void evolve(SimEventQueue& q);

  // show memory stats
  void showMemStats(const int frame, const size_t units) const;
};

// ######################################################################
//! SeC class to assist with checking for events
/*! This class is to allow for a simple syntax when checking for
  events:
  <PRE>

  // assume a SimEventQueue q
  if (SeC<SimEventBreak> e = q.check<SimEventBreak>(this))
    {
      // a matching event was found and is in 'e'
      cerr<<"Break: "<<e->toString()<<endl;
    }

  </PRE>

  With this syntax, matching event 'e' will only live within the scope
  of the 'if' statement. Note that this is a somewhat weird syntax and
  other seemingly equivalent syntaxes do not work. There must be an
  assignment and a provision to convert to bool for this to work. */
template <class T>
class SeC {
public:
  // Constructor; used by SimEventQueue::check()
  inline SeC(const rutz::shared_ptr<T> event) throw();

  // Assignment constructor
  inline SeC(const SeC<T>& other) throw();

  //! Destructor
  inline virtual ~SeC();

  //! Get the underlying event
  inline T* operator->() const throw();

  //! Get the underlying event
  inline T& operator*() const throw();

  //! Convert to bool by checking whether underlying event is valid
  /*! This is the key to enabling declaration of a SeC variable inside
    an if(...) statement. */
  inline operator bool() const;

  //! Check whether the pointee is non-null (there is an event)
  inline bool is_valid() const;

  //! Get the underlying shared_ptr
  inline rutz::shared_ptr<T>& getPtr();

  //! Get the underlying shared_ptr, const version
  inline const rutz::shared_ptr<T>& getPtr() const;

private:
  rutz::shared_ptr<T> itsEvent;

  // forbidden
  inline SeC() { };
};

// ######################################################################
// inlined functions:
// ######################################################################

// ######################################################################
template <class E> inline
void SimEventQueue::post(rutz::shared_ptr<E> e)
{
  // reject uninitialized shared_ptr's:
  if (e.is_valid() == false) LFATAL("You cannot post invalid events.");

  // enforce that E should derive from SimEvent, i.e., a pointer to an
  // object of type E should be dyn_cast'able to a pointer to type
  // SimEvent:
  rutz::shared_ptr<SimEvent> ebase; dyn_cast_to_from(ebase, e);
  if (ebase.is_valid() == false)
    LFATAL("Passed event of type '%s' does not derive from SimEvent", rutz::demangled_name(typeid(*e)));

  // plunge into the appropriate realm:
  if (e->itsSource == 0) LFATAL("Received event [%s] with invalid source", e->toString().c_str());
  SimEventQueue::RealmData *rd = getRealm(e->itsSource->realm(), false);
  SeqEntryVec *sevec = &(rd->events);
  scbm *cbmap = &(rd->callbacks);

  // let's delete all events from this caller that have same type as e and a time < t:
  SeqEntryVec::iterator ee = sevec->begin();
  while (ee != sevec->end())
    {
      rutz::shared_ptr<SimEvent> evt = ee->second;

      // is evt a shared_ptr onto a SimEvent derivative of type E,
      // coming from the same source as e, and with a time < tt?
      if (typeid(*evt) == typeid(*e) &&
          evt->itsSource == e->itsSource &&
          ee->first < t)
        ee = sevec->erase(ee);
      else ++ee;
    }

  // now push the new event into the queue, in case other people later
  // need it even though they don't have a callback registered for it
  // (they will have to manually do a check() for it), or in case some
  // of our callbacks below trigger other callbacks which in turn need
  // to check for our event::
  sevec->push_back(SeqEntry(t, e));

  // Call all our registered callbacks:
  scbm::iterator i = cbmap->find(&typeid(E));
  if (i != cbmap->end()) {
    // we have callbacks for this event type:
    rutz::shared_ptr<sscbb> cbset = i->second;

    // call them all in order (our sets are sorted):
    sscbb::iterator ii = cbset->begin(), stop = cbset->end();

    while (ii != stop) {
      // should we trigger the callback and how should we dispose of
      // the event? The logic here is similar to that in check() so
      // make sure both stay in sync if you modify one of them.
      // Differences include: the event here is brand new, just
      // posted, so clearly it is unmarked and nobody has seen it yet,
      // all clients will see it for the first time.
      const SimEventQueueFlag flags = (*ii)->sflags();
      const SimModule *eventsrc = (*ii)->eventsrc();
      SimCallbackClient *cli = (*ii)->client();
      SimModule *caller = dynamic_cast<SimModule*>(cli); // or 0 if caller not a SimModule

      // Only consider this event if either we con't care about a
      // specific event source, or we do care and the source matches:
      if (eventsrc == 0 || e->itsSource == eventsrc) {
        // run the callback function:
        LDEBUG("CALLBACK[%.2fms]: [%s]:%s, EVENT=[%s]", now().msecs(), e->itsSource->realm().c_str(),
               (*ii)->toString().c_str(), ebase->toString().c_str());

        (*ii)->execute(*this, ebase);

        // this event has been "consumed" by that caller, make a note of
        // that if caller was a full-blown SimModule:
        if (caller != 0 && (flags & SEQ_MARK)) e->markDone(caller);
      }
      ++ii;
    }
  }

  // let our helper know what happened:
  postHelper(typeid(E), e);
}

// ######################################################################
template <class R> inline
size_t SimEventQueue::request(rutz::shared_ptr<R> r, const SimReqQueueFlag flags)
{
  // reject uninitialized shared_ptr's:
  if (r.is_valid() == false) LFATAL("You cannot request invalid SimReqs.");

  // enforce that R should derive from SimReq, i.e., a pointer to an object of type R should be dyn_cast'able to a
  // pointer to type SimReq:
  rutz::shared_ptr<SimReq> rbase; dyn_cast_to_from(rbase, r);
  if (rbase.is_valid() == false)
    LFATAL("Passed request of type '%s' does not derive from SimReq", rutz::demangled_name(typeid(*r)));

  // plunge into the appropriate realm:
  if (r->itsSource == 0) LFATAL("Received req [%s] with invalid source", r->toString().c_str());
  SimEventQueue::RealmData *rd = getRealm(r->itsSource->realm(), false);
  srhm *rhmap = &(rd->reqhandlers);

  // Call all our registered handlers:
  size_t donesofar = 0;
  srhm::iterator i = rhmap->find(&typeid(R));
  if (i != rhmap->end())
    {
      // we have handlers for this request type:
      rutz::shared_ptr<ssrhb> rhset = i->second;

      // call them all in order:
      ssrhb::iterator ii = rhset->begin(), stop = rhset->end();

      while (ii != stop) {
        // Handle the request:
        const SimModule *reqsrc = (*ii)->reqsrc();

        // Only consider it if either we con't care about a specific req source, or we do care and the source matches:
        if (reqsrc == 0 || r->itsSource == reqsrc) {

          // run the handler function:
          LDEBUG("HANDLER[%.2fms]: [%s]:%s, REQ=[%s]", now().msecs(), r->itsSource->realm().c_str(),
                 (*ii)->toString().c_str(), rbase->toString().c_str());

          (*ii)->execute(*this, rbase);
          ++ donesofar;

          // do we want to enforce unique handling?
          if (donesofar > 1 && (flags & SRQ_SEVERAL_FATAL))
            LFATAL("More than one handler for %s requested by %s",
                   r->toString().c_str(),  r->itsSource->tagName().c_str());
        }
        ++ii;
      }
    }


  if (donesofar == 0 && (flags & SRQ_NONE_FATAL))
    LFATAL("No handler for %s requested by %s", r->toString().c_str(),  r->itsSource->tagName().c_str());

  // let our helper know what happened:
  requestHelper(typeid(R), r);

  return donesofar;
}

// ######################################################################
template <class E> inline
SeC<E> SimEventQueue::check(const SimModule* caller,
                            const SimEventQueueFlag flags,
                            const SimModule* eventsrc)
{
  // plunge into the appropriate realm:
  if (caller == 0) LFATAL("Received check request for [%s] from invalid caller", rutz::demangled_name(typeid(E)));
  SimEventQueue::RealmData *rd = getRealm(caller->realm(), false);
  SeqEntryVec *sevec = &(rd->events);

  SeqEntryVec::iterator e = sevec->begin();
  rutz::shared_ptr<E> event;

  while(e != sevec->end()) {
    // Only consider if either we con't care about a specific event source, or we do care and the source matches:
    if (eventsrc == 0 || e->second->itsSource == eventsrc) {
      // attempt a dynamic cast to type E:
      rutz::shared_ptr<E> ev = rutz::dyn_cast<E>(e->second);

      // if the dynamic cast succeeded, this event has the type we are looking for:
      if (ev.is_valid()) {
        // do we want to skip this event (if already marked, unless we don't care about marking)?
        if ((flags & SEQ_UNMARKED) == 0 || ev->isDone(caller) == false) {
          // ok, we want to consider this event as a match. Done searching.
          event = ev;
          break;
        }
      }
    }
    // keep searching...
    ++e;
  }

  // mark event as done if desired:
  if (event.is_valid() && (flags & SEQ_MARK)) event->markDone(caller);

  // let our helper know what happened:
  checkHelper(typeid(E), event, caller, flags, eventsrc);

  // if could not find a match, event is an empty shared_ptr:
  return SeC<E>(event);
}

// ######################################################################
inline void
SimEventQueue::checkHelper(const std::type_info& ename,
                           const rutz::shared_ptr<SimEvent>& e,
                           const SimModule* caller,
                           const SimEventQueueFlag flags,
                           const SimModule* eventsrc)
{ }

// ######################################################################
inline void
SimEventQueue::postHelper(const std::type_info& etype, const rutz::shared_ptr<SimEvent>& e)
{ }

// ######################################################################
inline void
SimEventQueue::requestHelper(const std::type_info& rtype, const rutz::shared_ptr<SimReq>& r)
{ }

// ######################################################################
template <class T>
inline SeC<T>::SeC(const rutz::shared_ptr<T> event) throw() :
  itsEvent(event)
{ }

template <class T>
inline SeC<T>::SeC(const SeC<T>& other) throw() :
  itsEvent(other.itsEvent)
{ }

template <class T> inline
SeC<T>::~SeC()
{ }

template <class T> inline
T* SeC<T>::operator->() const throw()
{ return itsEvent.get(); }

template <class T> inline
T& SeC<T>::operator*() const throw()
{ return *(itsEvent.get()); }

template <class T> inline
SeC<T>::operator bool() const
{ return itsEvent.is_valid(); }

template <class T> inline
bool SeC<T>::is_valid() const
{ return itsEvent.is_valid(); }

template <class T> inline
rutz::shared_ptr<T>& SeC<T>::getPtr()
{ return itsEvent; }

template <class T> inline
const rutz::shared_ptr<T>& SeC<T>::getPtr() const
{ return itsEvent; }


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // SIMULATION_SIMEVENTQUEUE_H_DEFINED
