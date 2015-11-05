/*!@file Simulation/SimCallback.H A callback when some SimEvent is received by SimEventQueue */

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
// Primary maintainer for this file: Laurent Itti <itti@pollux.usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Simulation/SimCallback.H $
// $Id: SimCallback.H 13065 2010-03-28 00:01:00Z itti $
//

#ifndef SIMULATION_SIMCALLBACK_H_DEFINED
#define SIMULATION_SIMCALLBACK_H_DEFINED

#include <typeinfo>
#include "rutz/shared_ptr.h"
#include "rutz/demangle.h"
#include "Simulation/SimEventQueueFlag.H"
#include "Util/log.H"
#include "Util/sformat.H"

class SimCallbackClient;
class SimEventQueue;
class SimEvent;
class SimModule;

//! A SimEventQueue callback for SimModule
/*! This is the base class. We will then create template derived
    classes (see ModelParamBase and ModelParam for a similar
    idea). The basic idea here is that a SimModule should, at
    construction, declare a series of callbacks which SimEventQueue
    will automatically call when the appropriate SimEvent objects are
    received by the queue. The standard C++ way of doing this
    typically would involve a callback class, and then derivatives of
    this, each one implementing the operator() in various ways
    (functor paradigm). However, this is not very convenient as
    typically the callback function would want to access the data
    members of its SimModule. Hence, we here use a different strategy,
    first declaring a function which is a member of SimModule and
    implements the actual code to be executed during callback, and
    second implementing a template "callback hook" whose role is just
    to connect that member function to the SimEventQueue.  SimCallback
    is just the hook. */
class SimCallbackBase {
public:
  //! Constructor
  SimCallbackBase(SimCallbackClient *smod, const int priority = 0,
                  const SimEventQueueFlag flags = SEQ_UNMARKED | SEQ_MARK,
                  const SimModule* eventsource = 0);

  //! Destructor
  virtual inline ~SimCallbackBase();

  //! What's our underlying SimEvent type?
  /*! Forbidden in the base class. */
  virtual const std::type_info& etype() const = 0;

  //! What's our underlying SimCallbackClient (usually SimModule) type?
  /*! Forbidden in the base class. */
  virtual const std::type_info& mtype() const = 0;

  //! Get the priority
  inline int priority() const;

  //! Get the SimCallbackClient
  virtual SimCallbackClient *client() const = 0;

  //! Get the flags
  inline const SimEventQueueFlag sflags() const;

  //! Get the desired event source, if any
  inline const SimModule* eventsrc() const;

  //! Get some string description of ourselves, for printing/debug
  inline std::string toString() const;

  //! Execute the callback
  /*! This will call the callback method on the callback client,
      passing it the queue and the event. */
  virtual void execute(SimEventQueue& q, rutz::shared_ptr<SimEvent>& e) = 0;

protected:
  int itsPriority;
  SimEventQueueFlag itsFlags;
  const SimModule *itsEventSrc;

private:
  SimCallbackBase(const SimCallbackBase& s);      //!< forbid copy-contruction
  SimCallbackBase& operator=(const SimCallbackBase& s); //!< forbid copy
};

// ######################################################################
//! Template class for SimEventQueue callbacks
/*! Method 'meth' will be called by the SimEventQueue when a
  desired event is received. Typical usage is as follows:

  in your .H:

  \code

  // your class should inherit from at least SimCallbackClient,
  // typically it will inherit from SimModule which itself iherits
  // from SimCallbackClient:
  class MySimModule : public SimModule {
  ...
  private:
  // our method to run when SimEventX is posted to the queue,
  // will be called by SimEventQueue:
  void processX(SimEventQueue& q, rutz::shared_ptr<SimEventX>& e);

  // a hook by which we will let our SimEventQueue know about processX():
  SimCallback<MySimModule, SimEventX> itsCBx;
  \endcode

  in your .C, in the constructor of your class (which must be
  a SimCallbackClient derivative), in the initalizer list:

  \code
  MySimModule::MySimModule() :
  ...
  itsCBx(this, &MySimModule::processX),
  ...
  { ... }
  \endcode

  and finally you also should implement MySimModule::processX(). */

template <class Module, class Event>
class SimCallback : public SimCallbackBase
{
public:
  //! Definition for the format and args of the called-back method
  typedef void (Module::*Method)(SimEventQueue&, rutz::shared_ptr<Event>&);

  //! Constructor
  inline SimCallback(Module* mod, Method meth, const int priority = 0,
                     const SimEventQueueFlag flags = SEQ_UNMARKED | SEQ_MARK,
                     const SimModule* eventsource = 0);

  //! Destructor
  virtual inline ~SimCallback();

  //! What's our underlying SimEvent type?
  virtual const std::type_info& etype() const;

  //! What's our underlying SimCallbackClient (usually SimModule) type?
  virtual const std::type_info& mtype() const;

  //! Get the SimCallbackClient
  virtual inline SimCallbackClient *client() const;

  //! Call the callback
  virtual inline void execute(SimEventQueue& q, rutz::shared_ptr<SimEvent>& e);

private:
  Module* itsModule;
  Method itsMethod;

  template <class M, class E> SimCallback(const SimCallback<M, E>& s);      //!< forbid copy-contruction
  template <class M, class E> SimCallback<Module, Event>& operator=(const SimCallback<M, E>& s); //!< forbid copy
};

// ######################################################################
//! Macro to declare a SimCallback (use in your .H)
/*! This will declare a SimCallback called itsCallback[eventname] and a
  function called on[eventname] which you will need to implement. For example:

  \code
  SIMCALLBACK_DECLARE(Retina, SimEventInputFrame);
  \endcode

  is equivalent to:

  \code
  void onSimEventInputFrame(SimEventQueue& q, rutz::shared_ptr<SimEventInputFrame>& e);
  SimCallback<Retina, SimEventInputFrame> itsCallbackSimEventInputFrame;
  \endcode

  then make sure you use the macro SIMCALLBACK_INIT in your class
  constructor, and that you implement onSimEventInputFrame() somewhere
  in your .C file. In our example:

  \code
  Retina::Retina(...) :
    ...,
    SIMCALLBACK_INIT(SimEventInputFrame)
  { }

  void Retina::onSimEventInputFrame(SimEventQueue& q, rutz::shared_ptr<SimEventInputFrame>& e)
  { do_something(); }
  \endcode

  Have a look at Retina and other modules in src/Neuro for examples of usage. */
#define SIMCALLBACK_DECLARE(classname, eventname)                             \
  void on##eventname(SimEventQueue& q, rutz::shared_ptr<eventname>& e);       \
  typedef classname itsSimCallbackClassTypeFor##eventname;                    \
  SimCallback<classname, eventname> itsCallback##eventname;

//! Init a SimCallback created by SIMCALLBACK_DECLARE
/*! Takes possible optional arguments which are the ones that have
    default values in the SimCallback constructor, namely:
      const int priority = 0,
      const SimEventQueueFlag flags = SEQ_UNMARKED | SEQ_MARK,
      const SimModule* eventsource = 0. */
#define SIMCALLBACK_INIT(eventname, ...)                                      \
  itsCallback##eventname(this, &itsSimCallbackClassTypeFor##eventname::on##eventname, ## __VA_ARGS__ )

// ######################################################################
//! Functor used to compare two SimCallbackBase objects, sorting by priority
struct SimCallbackCompare
{
  bool operator()(const SimCallbackBase *s1, const SimCallbackBase *s2) const
  { return (s1->priority() < s2->priority()); }
};


// ######################################################################
// Inline function implementations:
// ######################################################################
SimCallbackBase::~SimCallbackBase()
{ }

int SimCallbackBase::priority() const
{ return itsPriority; }

const SimEventQueueFlag SimCallbackBase::sflags() const
{ return itsFlags; }

const SimModule* SimCallbackBase::eventsrc() const
{ return itsEventSrc; }

std::string SimCallbackBase::toString() const
{
  return sformat("%s callback for %s [pri=%d]", rutz::demangled_name(this->etype()),
                 rutz::demangled_name(this->mtype()), this->priority());
}

template <class Module, class Event>
SimCallback<Module, Event>::
SimCallback(Module* mod, SimCallback<Module, Event>::Method meth,
            const int priority,  const SimEventQueueFlag flags,
            const SimModule* eventsource) :
  SimCallbackBase(mod, priority, flags, eventsource), itsModule(mod), itsMethod(meth)
{ }

template <class Module, class Event>
SimCallback<Module, Event>::~SimCallback()
{ }

template <class Module, class Event>
const std::type_info& SimCallback<Module, Event>::etype() const
{ return typeid(Event); }

template <class Module, class Event>
const std::type_info& SimCallback<Module, Event>::mtype() const
{ return typeid(Module); }

template <class Module, class Event>
SimCallbackClient* SimCallback<Module, Event>::client() const
{ return itsModule; }

template <class Module, class Event>
void SimCallback<Module, Event>::
execute(SimEventQueue& q, rutz::shared_ptr<SimEvent>& e)
{
  // Make sure e is of type Event:
  rutz::shared_ptr<Event> ee; dyn_cast_to_from(ee, e);
  if (ee.is_valid() == false)
    LFATAL("Passed SimEvent does not derive from '%s'", rutz::demangled_name(typeid(Event)));

  // want to learn a new exotic syntax? calling a pointer to member function
  (itsModule->*itsMethod)(q, ee);
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // SIMULATION_SIMCALLBACK_H_DEFINED
