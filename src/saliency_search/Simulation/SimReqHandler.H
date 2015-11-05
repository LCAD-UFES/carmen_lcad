/*!@file Simulation/SimReqHandler.H Handle SimReq requests */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Simulation/SimReqHandler.H $
// $Id: SimReqHandler.H 13065 2010-03-28 00:01:00Z itti $
//

#ifndef SIMULATION_SIMREQHANDLER_H_DEFINED
#define SIMULATION_SIMREQHANDLER_H_DEFINED
#include <typeinfo>
#include "rutz/shared_ptr.h"
#include "rutz/demangle.h"
#include "Simulation/SimEventQueueFlag.H"
#include "Util/log.H"
#include "Util/sformat.H"

class SimReqHandlerClient;
class SimEventQueue;
class SimReq;
class SimModule;

//! A SimEventQueue callback for SimModule
/*! This is the base class. We will then create template derived
    classes (see ModelParamBase and ModelParam for a similar
    idea). The basic idea here is that a SimModule should, at
    construction, declare a series of request handlers which
    SimEventQueue will automatically route when the appropriate SimReq
    objects are received by the queue. The standard C++ way of doing
    this typically would involve a callback class, and then
    derivatives of this, each one implementing the operator() in
    various ways (functor paradigm). However, this is not very
    convenient as typically the handler function would want to access
    the data members of its SimModule. Hence, we here use a different
    strategy, first declaring a function which is a member of
    SimModule and implements the actual code to be executed during
    callback, and second implementing a template "handler hook" whose
    role is just to connect that member function to the SimEventQueue.
    SimReqHandler is just the hook. */
class SimReqHandlerBase {
public:
  //! Constructor
  SimReqHandlerBase(SimReqHandlerClient *smod, const SimModule* reqsource = 0);

  //! Destructor
  virtual inline ~SimReqHandlerBase();

  //! What's our underlying SimReq type?
  /*! Forbidden in the base class. */
  virtual const std::type_info& rtype() const = 0;

  //! What's our underlying SimReqHandlerClient (usually SimModule) type?
  /*! Forbidden in the base class. */
  virtual const std::type_info& mtype() const = 0;

  //! Get the SimReqHandlerClient
  virtual SimReqHandlerClient *client() const = 0;

  //! Get the desired req source, if any
  inline const SimModule* reqsrc() const;

  //! Get some string description of ourselves, for printing/debug
  inline std::string toString() const;

  //! Execute the handler
  /*! This will call the handler method on the req handler client,
      passing it the queue and the req. */
  virtual void execute(SimEventQueue& q, rutz::shared_ptr<SimReq>& e) = 0;

protected:
  const SimModule *itsReqSrc;

private:
  SimReqHandlerBase(const SimReqHandlerBase& s);      //!< forbid copy-contruction
  SimReqHandlerBase& operator=(const SimReqHandlerBase& s); //!< forbid copy
};

// ######################################################################
//! Template class for SimEventQueue requests
/*! Method 'meth' will be called by the SimEventQueue when a
  desired requesr is triggered. Typical usage is as follows:

  in your .H:

  \code

  // your class should inherit from at least SimReqHandlerClient,
  // typically it will inherit from SimModule which itself iherits
  // from SimReqHandlerClient:
  class MySimModule : public SimModule {
  ...
  private:
  // our method to run when SimReqX is posted to the queue,
  // will be called by SimEventQueue:
  void processX(SimEventQueue& q, rutz::shared_ptr<SimReqX>& e);

  // a hook by which we will let our SimEventQueue know about processX():
  SimReqHandler<MySimModule, SimReqX> itsRHx;
  \endcode

  in your .C, in the constructor of your class (which must be
  a SimReqHandlerClient derivative), in the initalizer list:

  \code
  MySimModule::MySimModule() :
  ...
  itsRHx(this, &MySimModule::processX),
  ...
  { ... }
  \endcode

  and finally you also should implement MySimModule::processX(). */

template <class Module, class Req>
class SimReqHandler : public SimReqHandlerBase
{
public:
  //! Definition for the format and args of the called-back method
  typedef void (Module::*Method)(SimEventQueue&, rutz::shared_ptr<Req>&);

  //! Constructor
  inline SimReqHandler(Module* mod, Method meth, const SimModule* reqsource = 0);

  //! Destructor
  virtual inline ~SimReqHandler();

  //! What's our underlying SimReq type?
  virtual const std::type_info& rtype() const;

  //! What's our underlying SimReqHandlerClient (usually SimModule) type?
  virtual const std::type_info& mtype() const;

  //! Get the SimReqHandlerClient
  virtual inline SimReqHandlerClient *client() const;

  //! Call the callback
  virtual inline void execute(SimEventQueue& q, rutz::shared_ptr<SimReq>& e);

private:
  Module* itsModule;
  Method itsMethod;

  template <class M, class E> SimReqHandler(const SimReqHandler<M, E>& s);      //!< forbid copy-contruction
  template <class M, class E> SimReqHandler<Module, Req>& operator=(const SimReqHandler<M, E>& s); //!< forbid copy
};

// ######################################################################
//! Macro to declare a SimReqHandler (use in your .H)
/*! This will declare a SimReqHandler called itsReqHandler[reqname] and a
  function called handle[reqname] which you will need to implement. For example:

  \code
  SIMREQHANDLER_DECLARE(VisualCortex, SimReqVCXfeatures);
  \endcode

  is equivalent to:

  \code
  void handleSimReqVCXfeatures(SimEventQueue& q, rutz::shared_ptr<SimReqVCXfeatures>& e);
  SimReqHandler<VisualCortex, SimReqVCXfeatures> itsReqHandlerSimReqVCXfeatures;
  \endcode

  then make sure you use the macro SIMCALLBACK_INIT in your class
  constructor, and that you implement onSimReqInputFrame() somewhere
  in your .C file. In our example:

  \code
  VisualCortex::VisualCortex(...) :
    ...,
    SIMREQHANDLER_INIT(SimReqVCXfeatures)
  { }

  void VisualCortex::handleSimReqVCXfeatures(SimEventQueue& q, rutz::shared_ptr<SimReqVCXfeatures>& e)
  { do_something(); }
  \endcode

  Have a look at VisualCortex and other modules in src/Neuro for examples of usage. */
#define SIMREQHANDLER_DECLARE(classname, reqname)                             \
  void handle##reqname(SimEventQueue& q, rutz::shared_ptr<reqname>& e);       \
  typedef classname itsSimReqClassTypeFor##reqname;                           \
  SimReqHandler<classname, reqname> itsReqHandler##reqname;

//! Init a SimReqHandler created by SIMREQHANDLER_DECLARE
/*! Takes possible optional arguments which are the ones that have
    default values in the SimReqHandler constructor, namely:
      const SimModule* source = 0. */
#define SIMREQHANDLER_INIT(reqname, ...)                                      \
  itsReqHandler##reqname(this, &itsSimReqClassTypeFor##reqname::handle##reqname, ## __VA_ARGS__ )

// ######################################################################
// Inline function implementations:
// ######################################################################
SimReqHandlerBase::~SimReqHandlerBase()
{ }

const SimModule* SimReqHandlerBase::reqsrc() const
{ return itsReqSrc; }

std::string SimReqHandlerBase::toString() const
{
  return sformat("%s handler by %s", rutz::demangled_name(this->rtype()),
                 rutz::demangled_name(this->mtype()));
}

template <class Module, class Req>
SimReqHandler<Module, Req>::
SimReqHandler(Module* mod, SimReqHandler<Module, Req>::Method meth,
            const SimModule* source) :
  SimReqHandlerBase(mod, source), itsModule(mod), itsMethod(meth)
{ }

template <class Module, class Req>
SimReqHandler<Module, Req>::~SimReqHandler()
{ }

template <class Module, class Req>
const std::type_info& SimReqHandler<Module, Req>::rtype() const
{ return typeid(Req); }

template <class Module, class Req>
const std::type_info& SimReqHandler<Module, Req>::mtype() const
{ return typeid(Module); }

template <class Module, class Req>
SimReqHandlerClient* SimReqHandler<Module, Req>::client() const
{ return itsModule; }

template <class Module, class Req>
void SimReqHandler<Module, Req>::
execute(SimEventQueue& q, rutz::shared_ptr<SimReq>& e)
{
  // Make sure e is of type Req:
  rutz::shared_ptr<Req> ee; dyn_cast_to_from(ee, e);
  if (ee.is_valid() == false)
    LFATAL("Passed SimReq does not derive from '%s'", rutz::demangled_name(typeid(Req)));

  // want to learn a new exotic syntax? calling a pointer to member function
  (itsModule->*itsMethod)(q, ee);
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // SIMULATION_SIMREQHANDLER_H_DEFINED
