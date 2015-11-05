/**
   \file  Robots/LoBot/control/LoCountdown.C
   \brief This file defines the non-inline member functions of the
   lobot::Countdown class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoCountdown.C $
// $Id: LoCountdown.C 14305 2010-12-08 21:17:33Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoCountdown.H"
#include "Robots/LoBot/control/LoMetrics.H"

#include "Robots/LoBot/LoApp.H"
#include "Robots/LoBot/slam/LoMap.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/thread/LoShutdown.H"

#include "Robots/LoBot/misc/LoRegistry.H"
#include "Robots/LoBot/misc/singleton.hh"
#include "Robots/LoBot/util/LoTime.H"

// Standard C++ headers
#include <iomanip>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- KNOB TWIDDLING -----------------------------

namespace {

// Retrieve settings from monitor_danger_zone section of config file
template<typename T>
inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>(LOBE_COUNTDOWN, key, default_value) ;
}

/// This local class encapsulates various parameters that can be used to
/// tweak different aspects of the monitor_danger_zone behaviour.
class CountdownParams : public singleton<CountdownParams> {
   /// This setting specifies the number of seconds for the countdown.
   int m_duration ;

   /// The number of milliseconds between successive iterations of this
   /// behaviour.
   ///
   /// WARNING: The ability to change a behaviour's update frequency is a
   /// very powerful feature whose misuse or abuse can wreak havoc! Be
   /// sure to use reasonable values for this setting.
   int m_update_delay ;

   /// Private constructor because this is a singleton.
   CountdownParams() ;
   friend class singleton<CountdownParams> ;

public:
   /// Accessing the various parameters
   //@{
   static int duration()     {return instance().m_duration     ;}
   static int update_delay() {return instance().m_update_delay ;}
   //@}
} ;

// Parameter initialization
CountdownParams::CountdownParams()
   : m_duration(clamp(conf("duration", 10800), 5, 86400) * 1000),
     m_update_delay(clamp(conf("update_delay", 1000), 500, 5000))
{}

// Shortcut
typedef CountdownParams Params ;

} // end of local anonymous namespace encapsulating above helpers

//-------------------------- INITIALIZATION -----------------------------

Countdown::Countdown()
   : base(Params::update_delay(), LOBE_COUNTDOWN),
     m_time(-1)
{
   start(LOBE_COUNTDOWN) ;
}

void Countdown::pre_run()
{
   m_time = current_time() ;
}

//---------------------- THE BEHAVIOUR'S ACTION -------------------------

void Countdown::action()
{
   if (current_time() - m_time >= Params::duration())
   {
      using std::setw ; using std::left ;

      Metrics::Log log ;
      log << setw(Metrics::opw()) << left << base::name ;
      Map* M = App::map() ;
      if (M)
         log << M->current_pose() ;

      Shutdown::signal() ;
   }
}

//----------------------------- CLEAN-UP --------------------------------

Countdown::~Countdown(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
