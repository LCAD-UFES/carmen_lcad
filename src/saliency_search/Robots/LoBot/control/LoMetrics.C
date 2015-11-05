/**
   \file  Robots/LoBot/control/LoMetrics.C
   \brief This file defines the static data members and non-inline member
   functions of the lobot::Metrics class.
*/

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
// Primary maintainer for this file: mviswana usc edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoMetrics.C $
// $Id: LoMetrics.C 13838 2010-08-27 20:42:20Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoMetrics.H"

#include "Robots/LoBot/LoApp.H"
#include "Robots/LoBot/slam/LoMap.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"

#include "Robots/LoBot/misc/LoRegistry.H"
#include "Robots/LoBot/misc/singleton.hh"
#include "Robots/LoBot/util/LoTime.H"

// Standard C++ headers
#include <fstream>
#include <algorithm>
#include <iterator>

// Standard C headers
//#include <time.h>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- KNOB TWIDDLING -----------------------------

namespace {

// Retrieve settings from vfh section of config file
template<typename T>
inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>(LOBE_METRICS, key, default_value) ;
}

/// This local class encapsulates various parameters that can be used to
/// tweak different aspects of the metrics behaviour.
class Params : public singleton<Params> {
   /// This setting specifies the prefix of the file name to which the
   /// metrics should be written. By default, it is
   /// "/tmp/lobot-metrics-".
   ///
   /// The remainder of the metrics log file's name is generated using
   /// the current time in the format "yyyymmdd-HHMMSS". There is no file
   /// name extension.
   std::string m_log_file ;

   /// The metrics behaviour buffers all the metrics related messages
   /// sent by other behaviours and, periodically, when the buffer is
   /// "full," dumps the buffer to the log file. This setting specifies
   /// the minimum number of messages that should be collected before the
   /// internal buffer will be written to disk and cleared.
   int m_dump_threshold ;

   /// Most log messages begin with a small descriptive phrase such as
   /// "tracking pose" and are then followed by some values (such as the
   /// pose alluded to in the afore-mentioned phrase). To make the log
   /// more readable, all the opening phrases are written left-justified
   /// in a field width specified by this setting.
   int m_opening_width ;

   /// The number of milliseconds between successive iterations of this
   /// behaviour.
   ///
   /// WARNING: The ability to change a behaviour's update frequency is a
   /// very powerful feature whose misuse or abuse can wreak havoc! Be
   /// sure to use reasonable values for this setting.
   int m_update_delay ;

   /// Private constructor because this is a singleton.
   Params() ;
   friend class singleton<Params> ;

public:
   /// Accessing the various parameters
   //@{
   static const std::string& log_file() {return instance().m_log_file ;}
   static int dump_threshold() {return instance().m_dump_threshold ;}
   static int opening_width()  {return instance().m_opening_width  ;}
   static int update_delay()   {return instance().m_update_delay   ;}
   //@}
} ;

// Parameter initialization
Params::Params()
   : m_log_file(conf<std::string>("log_prefix", "/tmp/lobot-metrics-")
                   + startup_timestamp_str()),
     m_dump_threshold(clamp(conf("dump_threshold", 500), 100, 5000)),
     m_opening_width(clamp(conf("opening_phrase_width", 20), 16, 40)),
     m_update_delay(clamp(conf("update_delay", 2500), 750, 5000))
{}

} // end of anonymous local namespace encapsulating above helpers

//------------------------ STATIC DATA MEMBERS --------------------------

// As explained in the header file, this class is not really a singleton,
// but acts like one nonetheless.
Metrics* Metrics::m_instance ;

//-------------------------- INITIALIZATION -----------------------------

Metrics::Metrics()
   : base(Params::update_delay())
{
   m_instance = this ;
   start(LOBE_METRICS) ;
}

// Before we start up, let's allocate enough memory for the messages
// buffer. But no point wasting memory in case there are multiple
// instances of this behaviour (should not really happen in practice).
void Metrics::pre_run()
{
   if (m_instance == this) {
      const int T = Params::dump_threshold() ;
      m_buffer.reserve(3*T/2) ;
   }
}

//---------------------- THE BEHAVIOUR'S ACTION -------------------------

// Quick helper to dump message buffer to log file
static void dump(const std::vector<std::string>& messages)
{
   std::ofstream log(Params::log_file().c_str(),
                     std::ios::out | std::ios::app) ;
   std::copy(messages.begin(), messages.end(),
             std::ostream_iterator<std::string>(log, "\n")) ;
}

// Periodically dump message buffer to log file
void Metrics::action()
{
   // Usually, there should be just one instance of the metrics
   // behaviour. But just in case the config file somehow asks for
   // multiple instances of this behaviour, we should double-check and
   // ignore the action methods for all but the "active" metrics
   // behaviour. There ought be no real harm in not performing this
   // short-circuit test; but no point executing useless code...
   if (m_instance != this)
      return ;

   // Copy message buffer to avoid holding up other threads using the
   // metrics logging facility.
   std::vector<std::string> messages ;
   m_mutex.acquire() ;
      if (static_cast<int>(m_buffer.size()) >= Params::dump_threshold()) {
         std::copy(m_buffer.begin(), m_buffer.end(),
                   std::back_inserter(messages)) ;
         m_buffer.clear() ;
      }
   m_mutex.release() ;
   dump(messages) ;
}

//----------------------------- CLEAN-UP --------------------------------

// When app is quitting, dump remaining messages in buffer to log.
//
// DEVNOTE: The destructor should be invoked from lobot::App::~App(). At
// that point, all threads should be dead. Therefore, it should be safe
// to use the buffer without the mutex.
Metrics::~Metrics()
{
   if (m_instance == this) {
      dump(m_buffer) ;
      m_instance = 0 ;
   }
}

//---------------------------- PUBLIC API -------------------------------

// Helper function for prefixing log messages with a time-stamp
void Metrics::Log::ts()
{
   str << current_time() << ' ' ;
}

// Helper function for queuing log messages in the metrics behaviour's
// internal message buffer.
void Metrics::Log::send()
{
   Metrics* M = Metrics::m_instance ;
   if (M)
   {
      AutoMutex m(M->m_mutex) ;
      M->m_buffer.push_back(str.str()) ;
   }
}

// The current time is automatically recorded with every metric
Metrics::Log::Log()
   : need_ts(true)
{}

// When the local Metrics::Log object in the client function goes out of
// scope, the message it has accumulated will be added to the Metrics
// object's internal buffer.
Metrics::Log::~Log()
{
   send() ;
}

// When clients want to "reuse" a local Metrics::Log object over and over
// to create new messages, they should use a Metrics::endl manipulator to
// end one message and begin another.
Metrics::Log& operator<<(Metrics::Log& metlog, const Metrics::endl&)
{
   metlog.send() ;
   metlog.str.str("") ;
   metlog.need_ts = true ;
   return metlog ;
}

// To split a long message across multiple lines
Metrics::Log& operator<<(Metrics::Log& metlog, const Metrics::newl&)
{
   metlog.str << " \\\n        " ;
   return metlog ;
}

// Helper to return the opening phrase's field width for each log message
int Metrics::opening_phrase_width()
{
   return Params::opening_width() ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
