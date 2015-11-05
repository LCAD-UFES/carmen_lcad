/*!@file Util/TextLog.C Functions for logging model/simulation events */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/TextLog.C $
// $Id: TextLog.C 6528 2006-04-25 18:57:30Z rjpeters $
//

#ifndef UTIL_TEXTLOG_C_DEFINED
#define UTIL_TEXTLOG_C_DEFINED

#include "Util/TextLog.H"

#include "Util/Assert.H"
#include "Util/SimTime.H"
#include "Util/log.H"
#include "Util/sformat.H"
#include "rutz/mutex.h"

#include <fstream>
#include <map>

/* A note on the implementation strategy here for the text-logging
   functions: internally, we keep a map of TextLogger objects, one per
   filename. When one of the public functions is called, we look up
   the associated TextLogger object, creating it if needed, and then
   perform the requested logging action on that object.
*/

namespace
{
  // ######################################################################
  class TextLogger
  {
  public:
    TextLogger(const std::string& fname)
      :
      itsMutex(),
      itsStream(fname.c_str()),
      itsTime(SimTime::ZERO())
    {
      pthread_mutex_init(&itsMutex, 0);

      if (!itsStream.is_open())
        {
          LFATAL("Couldn't open text log file '%s' for writing",
                 fname.c_str());
        }
      else
        {
          LINFO("Opened textlog file '%s'", fname.c_str());
        }
    }

    void log(const std::string& event,
             const std::string& details)
    {
      this->log(event, details, itsTime);
    }

    void log(const std::string& event,
             const std::string& details,
             const SimTime& t)
    {
      GVX_MUTEX_LOCK(&itsMutex);

      itsStream
        << sformat("%010.02fms %-20s ", t.msecs(), event.c_str())
        << details << std::endl;
      itsStream.flush();  // don't lose messages if ctrl-C
    }

    void setTime(const SimTime& t)
    {
      GVX_MUTEX_LOCK(&itsMutex);

      itsTime = t;
    }

  private:
    pthread_mutex_t itsMutex;
    std::ofstream itsStream;
    SimTime itsTime;
  };

  // ######################################################################
  typedef std::map<std::string, TextLogger*> LoggerMap;

  LoggerMap* g_loggers;
  pthread_mutex_t g_loggers_mutex = PTHREAD_MUTEX_INITIALIZER;

  pthread_once_t g_loggers_init_once = PTHREAD_ONCE_INIT;
  void loggers_init()
  {
    ASSERT(g_loggers == 0);
    g_loggers = new LoggerMap;
  }

  // ######################################################################
  TextLogger* get_logger(const std::string& fname)
  {
    pthread_once(&g_loggers_init_once, loggers_init);

    ASSERT(g_loggers != 0);

    GVX_MUTEX_LOCK(&g_loggers_mutex);

    LoggerMap::iterator itr = g_loggers->find(fname);

    if (itr != g_loggers->end())
      {
        return (*itr).second;
      }

    // else, let's create a new logger for the given fname
    TextLogger* logger = new TextLogger(fname);
    g_loggers->insert(LoggerMap::value_type(fname, logger));
    return logger;
  }
}

// ######################################################################
void textLog(const std::string& fname,
             const std::string& event,
             const std::string& details)
{
  if (!fname.empty())
    get_logger(fname)->log(event, details);
}

// ######################################################################
void textLog(const std::string& fname,
             const std::string& event,
             const std::string& details,
             const SimTime& t)
{
  if (!fname.empty())
    get_logger(fname)->log(event, details, t);
}

// ######################################################################
void setLogTime(const std::string& fname,
                const SimTime& t)
{
  if (!fname.empty())
    get_logger(fname)->setTime(t);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_TEXTLOG_C_DEFINED
