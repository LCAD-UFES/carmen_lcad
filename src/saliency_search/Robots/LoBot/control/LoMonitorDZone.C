/**
   \file  Robots/LoBot/control/LoMonitorDZone.C
   \brief This file defines the non-inline member functions of the
   lobot::MonitorDZone class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoMonitorDZone.C $
// $Id: LoMonitorDZone.C 14041 2010-09-25 02:10:39Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoMonitorDZone.H"
#include "Robots/LoBot/control/LoMetrics.H"
#include "Robots/LoBot/control/LoSpinArbiter.H"

#include "Robots/LoBot/LoApp.H"
#include "Robots/LoBot/slam/LoMap.H"
#include "Robots/LoBot/io/LoDangerZone.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"

#include "Robots/LoBot/thread/LoUpdateLock.H"
#include "Robots/LoBot/thread/LoPause.H"
#include "Robots/LoBot/thread/LoShutdown.H"

#include "Robots/LoBot/misc/LoRegistry.H"
#include "Robots/LoBot/misc/singleton.hh"

#include "Robots/LoBot/util/LoString.H"
#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/util/LoTime.H"

// Standard C++ headers
#include <iomanip>
#include <algorithm>
#include <iterator>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- KNOB TWIDDLING -----------------------------

namespace {

// Retrieve settings from monitor_danger_zone section of config file
template<typename T>
inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>(LOBE_MONITOR_DZONE, key, default_value) ;
}

// This enumeration names the different possible actions the
// monitor_danger_zone behaviour can take when it finds that the robot's
// danger zone has been penetrated.
enum Action {
   SPIN,
   PAUSE,
   QUIT,
} ;

/// This local class encapsulates various parameters that can be used to
/// tweak different aspects of the monitor_danger_zone behaviour.
class MonitorDZoneParams : public singleton<MonitorDZoneParams> {
   /// Once the robot's danger zone has been penetrated, the
   /// monitor_danger_zone behaviour will start an internal stopwatch
   /// ticking and wait for it to reach at least the amount specified by
   /// this setting before actually taking any action.
   ///
   /// This setting should be a time delay specified in milliseconds.
   int m_duration ;

   /// By default, the monitor_danger_zone behaviour will wait for the
   /// robot to come to a full stop before considering any action. Thus,
   /// if the danger zone has been penetrated but the robot is still
   /// moving (for example, an extricate behaviour is getting the robot
   /// unstuck), then monitor_danger_zone will not interfere.
   ///
   /// However, by turning this flag off, we can have this behaviour
   /// ignore the robot's current state of motion so that it takes action
   /// whenever the danger zone has been penetrated for at least the
   /// duration specified by the previous setting regardless of whether
   /// the robot is moving or not.
   bool m_wait_for_stop ;

   /// This setting specifies the action to take when the robot's danger
   /// zone has been penetrated. The following actions are supported:
   ///
   ///    - Spin, i.e., turn the robot in-place, by some random amount.
   ///      This is the default action.
   ///
   ///    - Pause the robot and wait for the user to take appropriate
   ///      action, e.g., remote control to get the robot out of the
   ///      danger zone.
   ///
   ///      NOTE: The pause action will require the user to explicitly
   ///      restart the robot's controller, i.e., unpause it by pressing
   ///      the 'p' key. This also requires that the UI not be disabled.
   ///
   ///    - Quit the Robolocust controller application. This can be
   ///      useful, for instance, when we are running experiments that
   ///      require us to simply drive the robot up to a wall and then
   ///      stop and start over.
   Action m_action ;

   /// The number of milliseconds between successive iterations of this
   /// behaviour.
   ///
   /// WARNING: The ability to change a behaviour's update frequency is a
   /// very powerful feature whose misuse or abuse can wreak havoc! Be
   /// sure to use reasonable values for this setting.
   int m_update_delay ;

   /// Private constructor because this is a singleton.
   MonitorDZoneParams() ;
   friend class singleton<MonitorDZoneParams> ;

public:
   /// Accessing the various parameters
   //@{
   static int    duration()      {return instance().m_duration      ;}
   static bool   wait_for_stop() {return instance().m_wait_for_stop ;}
   static Action action()        {return instance().m_action        ;}
   static int    update_delay()  {return instance().m_update_delay  ;}
   //@}
} ;

// Parameter initialization
MonitorDZoneParams::MonitorDZoneParams()
   : m_duration(clamp(conf("duration", 2500), 100, 60000)),
     m_wait_for_stop(conf("wait_for_stop", true)),
     m_update_delay(clamp(conf("update_delay", 1000), 100, 5000))
{
   const std::string action = downstring(conf<std::string>("action", "spin")) ;
   if (action == "spin")
      m_action = SPIN ;
   else if (action == "pause")
      m_action = PAUSE ;
   else if (action == "quit")
      m_action = QUIT ;
}

// Shortcut
typedef MonitorDZoneParams Params ;

} // end of local anonymous namespace encapsulating above helpers

//-------------------------- INITIALIZATION -----------------------------

MonitorDZone::MonitorDZone()
   : base(Params::update_delay(), LOBE_MONITOR_DZONE),
     m_time(-1)
{
   start(LOBE_MONITOR_DZONE) ;
}

//---------------------- THE BEHAVIOUR'S ACTION -------------------------

static void log(const std::string& msg)
{
   using std::setw ; using std::left ;

   Metrics::Log log ;
   log << setw(Metrics::opw()) << left << msg ;
   Map* M = App::map() ;
   if (M)
      log << M->current_pose() ;
}

static void spin(int angle)
{
   log(std::string("mon_dzone spin ") + to_string(angle)) ;
   SpinArbiter::instance().
      vote(LOBE_MONITOR_DZONE, new SpinArbiter::Vote(angle)) ;
}

void MonitorDZone::action()
{
   UpdateLock::begin_read() ;
      bool danger_zone_penetrated = DangerZone::penetrated() ;
      bool stopped = App::robot()->stopped() ;
   UpdateLock::end_read() ;

   if (danger_zone_penetrated)
   {
      if (m_time < 0)
      {
         if (Params::wait_for_stop() && !stopped)
            return ;
         m_time = current_time() ;
         log("mon_dzone begin") ;
      }
      else
      {
         if (Params::wait_for_stop() && !stopped)
            reset() ;
         else
         {
            int duration = current_time() - m_time ;
            if (duration >= Params::duration())
            {
               switch (Params::action())
               {
                  case SPIN:
                     spin(random(-350, 350)) ;
                     break ;
                  case PAUSE:
                     log("mon_dzone pause") ;
                     Pause::set() ;
                     break ;
                  case QUIT:
                     log("mon_dzone quit") ;
                     Shutdown::signal() ;
                     break ;
               }
               reset() ;
            }
         }
      }
   }
   else
   {
      if (m_time >= 0)
         reset() ;
   }
}

void MonitorDZone::reset()
{
   m_time = -1 ;
   log("mon_dzone end") ;
}

//----------------------------- CLEAN-UP --------------------------------

MonitorDZone::~MonitorDZone(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
