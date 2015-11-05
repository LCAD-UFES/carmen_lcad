/*!@file Component/EventLog.H Simple text-based event logger */

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
// Primary maintainer for this file:
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/EventLog.H $
// $Id: EventLog.H 14942 2011-10-04 00:30:24Z dberg $
//

#ifndef COMPONENT_EVENTLOG_H_DEFINED
#define COMPONENT_EVENTLOG_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Util/Timer.H"
#include <list>
#include <pthread.h>

//! EventLog is a simple text-based event logger
/*! EventLog allows simple logging of various user-defined events,
  with simultaneous recording of the (real) time at which the events
  were triggered. This functionality used to be part of
  PsychoDisplay. EventLog derives from ModelComponent and hence can be
  configured via the command line. EventLog complements at least two
  types of logging available in this toolkit: in Util/log.H, functions
  are defined which can generate logs onto stderr or syslog,
  prepending class and function name to any event. In addition,
  ModelComponent itself has a built-int logging facility that logs
  events to file without a timestamp (see --txtlog command-line
  option). One critical feature of EventLog is its ability to keep all
  logs in a mmory queue util the program is stopped, hence avoiding
  disk access during execution of a program. */
class EventLog : public ModelComponent
{
public:
  //! Constructor
  EventLog(OptionManager& mgr,
           const std::string& descrName = "Event Log",
           const std::string& tagName = "EventLog");

  //! Destructor
  virtual ~EventLog();

  //! push a new event into the event queue
  void pushEvent(const std::string& msg);

  //! push the beginning of an event into our queue
  /*! This is like pushEvent() but it will add a start marker to the string */
  void pushEventBegin(const std::string& msg);

  //! push the end of an event into our queue
  /*! This is like pushEvent() but it will add an end marker to the string */
  void pushEventEnd(const std::string& msg);

  //! get the filename of the eventlog
  std::string getFileName();

protected:
  OModelParam<std::string> itsFileName; //!< filename for event log output

  virtual void start1(); //!< get started
  virtual void stop2();  //!< get stopped

private:
  // this is the data we record as event:
  struct LogEvent
  {
    uint64 tim;           // the time in microseconds since start()
    std::string descrip;  // description of the event
  };

  std::list<LogEvent> itsEvents;  // record a bunch of events
  Timer itsTimer;                 // Measure time since start()
  pthread_mutex_t itsMutex;       // make sure we are thread-safe
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // COMPONENT_EVENTLOG_H_DEFINED
