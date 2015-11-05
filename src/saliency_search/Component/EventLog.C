/*!@file Component/EventLog.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/EventLog.C $
// $Id: EventLog.C 14942 2011-10-04 00:30:24Z dberg $
//

#ifndef COMPONENT_EVENTLOG_C_DEFINED
#define COMPONENT_EVENTLOG_C_DEFINED

#include "Component/EventLog.H"
#include "Component/OptionManager.H"
#include "Component/ComponentOpts.H"
#include "Util/sformat.H"
#include <fstream>

// ######################################################################
EventLog::EventLog(OptionManager& mgr, const std::string& descrName,
                   const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsFileName(&OPT_EventLogFileName, this),
  itsEvents(), itsTimer(1000000)
{
  pthread_mutex_init(&itsMutex, NULL);
}

// ######################################################################
EventLog::~EventLog()
{
  pthread_mutex_destroy(&itsMutex);
}

// ######################################################################
void EventLog::start1()
{
  pthread_mutex_lock(&itsMutex);

  // clear events:
  itsEvents.clear();

  // reset timer:
  itsTimer.reset();

  pthread_mutex_unlock(&itsMutex);
}

// ######################################################################
void EventLog::stop2()
{
  pthread_mutex_lock(&itsMutex);

  // let's save the events:
  if (itsEvents.empty() == false && itsFileName.getVal().length() > 1)
    {
      const char *fname = itsFileName.getVal().c_str();
      std::ofstream ofs(fname);
      if (!ofs.is_open())
        LERROR("Couldn't open file '%s' for writing.", fname);
      else
        {
          std::list<LogEvent>::const_iterator itr = itsEvents.begin();
          while (itr != itsEvents.end())
            {
              const uint64 t = itr->tim;
              const int usec = int(t % 1000ULL);
              const int msec = int((t / 1000ULL) % 1000ULL);
              const int sec  = int((t / 1000000ULL) % 60ULL);
              const int minu = int((t / 60000000ULL) % 60ULL);
              const int hour = int(t / 3600000000ULL);

              ofs << sformat("%03d:%02d:%02d.%03d.%03d",
                             hour, minu, sec, msec, usec)
                  << " " << itr->descrip << std::endl;
              ++ itr;
            }
          ofs.close();
          LINFO("Saved log to '%s'", fname);
        }
    }

  pthread_mutex_unlock(&itsMutex);
}

// ######################################################################
void EventLog::pushEvent(const std::string& msg)
{
  LogEvent evt; evt.tim = itsTimer.get(); evt.descrip = msg;

  pthread_mutex_lock(&itsMutex);
  itsEvents.push_back(evt);
  pthread_mutex_unlock(&itsMutex);
}

// ######################################################################
void EventLog::pushEventBegin(const std::string& msg)
{ pushEvent(std::string("Begin: ") + msg); }

// ######################################################################
void EventLog::pushEventEnd(const std::string& msg)
{ pushEvent(std::string("End  : ") + msg); }

// ######################################################################
std::string EventLog::getFileName()
{ return itsFileName.getVal(); }

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // COMPONENT_EVENTLOG_C_DEFINED
