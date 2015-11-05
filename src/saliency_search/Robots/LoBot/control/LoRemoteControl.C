/**
   \file  Robots/LoBot/control/LoRemoteControl.C
   \brief This file defines the non-inline member functions of the
   lobot::RemoteControl class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoRemoteControl.C $
// $Id: LoRemoteControl.C 13521 2010-06-06 14:23:03Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoRemoteControl.H"
#include "Robots/LoBot/control/LoTurnArbiter.H"
#include "Robots/LoBot/control/LoSpeedArbiter.H"

#include "Robots/LoBot/LoApp.H"
#include "Robots/LoBot/io/LoRobot.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/thread/LoUpdateLock.H"

#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/LoRegistry.H"
#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/util/LoTime.H"

#include "Robots/LoBot/irccm/LoOpenInterface.h"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//--------------------------- LOCAL HELPERS -----------------------------

// Retrieve settings from extricate section of config file
template<typename T>
static inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>(LOBE_REMOTE_CONTROL, key, default_value) ;
}

// Freeze the arbiters to the given behaviour's priority so that
// behaviours with lower priority will be ignored.
static inline void freeze_arbiters(const std::string& name)
{
    TurnArbiter::instance().freeze(name) ;
   SpeedArbiter::instance().freeze(name) ;
}

// Unfreeze the arbiters so that other behaviours can resume having their
// actions processed.
static inline void unfreeze_arbiters(const std::string& name)
{
    TurnArbiter::instance().unfreeze(name) ;
   SpeedArbiter::instance().unfreeze(name) ;
}

// Returns true if the remote control command code received from the
// robot's sensors corresponds to one of the buttons this behaviour
// recognizes.
static bool supported_command(int c)
{
   switch (c)
   {
      case LOBOT_OI_REMOTE_LEFT:
      case LOBOT_OI_REMOTE_RIGHT:
      case LOBOT_OI_REMOTE_PAUSE:
      case LOBOT_OI_REMOTE_CLEAN:
      case LOBOT_OI_REMOTE_FORWARD:
         return true ;
      default:
         return false ;
   }
}

//-------------------------- INITIALIZATION -----------------------------

RemoteControl::RemoteControl()
   : base(clamp(conf("update_delay", 500), 1, 2500)),
     m_state(AUTONOMOUS),
     m_time(0)
{
   start(LOBE_REMOTE_CONTROL) ;
}

void RemoteControl::pre_run()
{
   if (! App::robot())
      throw behavior_error(MOTOR_SYSTEM_MISSING) ;
}

//---------------------- THE BEHAVIOUR'S ACTION -------------------------

// The remote control behaviour works by checking the robot's sensors to
// see if a remote control command has been sent. If so, it transitions
// to the REMOTE_CONTROL state and executes the specified command. While
// in the REMOTE_CONTROL state, if no more remote control commands come
// in within the configured timeout, the behaviour releases the arbiters
// and returns to the AUTONOMOUS state.
void RemoteControl::action()
{
   UpdateLock::begin_read() ;
      int ir = App::robot()->sensors().infrared() ; // remote control IR byte
   UpdateLock::end_read() ;

   switch (m_state)
   {
      case AUTONOMOUS:
         if (supported_command(ir))
         {
            freeze_arbiters(base::name) ;
            execute(ir) ;
            m_state = REMOTE_CONTROL ;
            m_time  = current_time() ;
         }
         break ;
      case REMOTE_CONTROL:
         if (current_time() - m_time <= Params::timeout())
         {
            if (supported_command(ir))
            {
               execute(ir) ;
               m_time = current_time() ;
            }
         }
         else // timed out without getting (valid) remote control command
         {
            m_state = AUTONOMOUS ;
            m_time  = 0 ;
            unfreeze_arbiters(base::name) ;
         }
         break ;
   }
}

// Execute the command specified by the remote control
void RemoteControl::execute(int cmd)
{
   switch (cmd)
   {
      case LOBOT_OI_REMOTE_FORWARD:
         TurnArbiter::instance().vote(base::name,
            new TurnArbiter::Vote(turn_vote_centered_at(0))) ;
         SpeedArbiter::instance().vote(base::name,
            new SpeedArbiter::Vote(Params::drive_speed(), 0)) ;
         break ;

      case LOBOT_OI_REMOTE_CLEAN: // use clean button for driving backwards
         TurnArbiter::instance().vote(base::name,
            new TurnArbiter::Vote(turn_vote_centered_at(0))) ;
         SpeedArbiter::instance().vote(base::name,
            new SpeedArbiter::Vote(-Params::drive_speed(), 0)) ;
         break ;

      case LOBOT_OI_REMOTE_PAUSE:
         TurnArbiter::instance().vote(base::name,
            new TurnArbiter::Vote(turn_vote_centered_at(0))) ;
         SpeedArbiter::instance().vote(base::name,
            new SpeedArbiter::Vote(0, 0)) ;
         break ;

      case LOBOT_OI_REMOTE_LEFT:
         TurnArbiter::instance().vote(base::name, new TurnArbiter::Vote(
            turn_vote_centered_at(TurnArbiter::turn_max()))) ;
         SpeedArbiter::instance().vote(base::name,
            new SpeedArbiter::Vote(Params::turn_speed(), 0)) ;
         break ;

      case LOBOT_OI_REMOTE_RIGHT:
         TurnArbiter::instance().vote(base::name, new TurnArbiter::Vote(
            turn_vote_centered_at(-TurnArbiter::turn_max()))) ;
         SpeedArbiter::instance().vote(base::name,
            new SpeedArbiter::Vote(Params::turn_speed(), 0)) ;
         break ;
   }
}

//----------------------------- CLEAN-UP --------------------------------

RemoteControl::~RemoteControl(){}

//-------------------------- KNOB TWIDDLING -----------------------------

// Parameters initialization
RemoteControl::Params::Params()
   : m_timeout(clamp(conf("timeout", 1000), 100, 5000)),
     m_drive_speed(clamp(conf("drive_speed", 0.3f), 0.1f, 0.5f)),
     m_turn_speed (clamp(conf("turn_speed",  0.2f), 0.1f, 0.5f))
{}

// Parameters clean-up
RemoteControl::Params::~Params(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
