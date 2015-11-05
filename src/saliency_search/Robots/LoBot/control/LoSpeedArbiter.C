/**
   \file  Robots/LoBot/control/LoSpeedArbiter.C
   \brief This file defines the non-inline member functions of the
   lobot::SpeedArbiter class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoSpeedArbiter.C $
// $Id: LoSpeedArbiter.C 13521 2010-06-06 14:23:03Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoSpeedArbiter.H"

#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/thread/LoUpdateLock.H"

#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/util/LoMath.H"

// INVT utilities
#include "Util/log.H"

// Standard C++ headers
#include <algorithm>

//------------------------------ MACROS ---------------------------------

// As a debugging aid, it can be useful to see how long an arbiter holds
// on to a vote before forwarding it to the motor system. This symbol can
// be turned on/off to enable/disable printing the above-mentioned
// vote-to-motor delay.
//#define LOBOT_PRINT_VOTE_TO_MOTOR_DELAY 1

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//--------------------------- LOCAL HELPERS -----------------------------

// Retrieve settings from speed_arbiter section of config file
template<typename T>
static inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>("speed_arbiter", key, default_value) ;
}

//-------------------------- INITIALIZATION -----------------------------

SpeedArbiter::SpeedArbiter()
   : Arbiter(clamp(conf("update_delay", 500), 1, 1000))
{
   start("speed_arbiter") ;
}

float SpeedArbiter::get_configured_priority(const std::string& behaviour) const
{
   return abs(get_conf(behaviour, "speed_priority", 0.0f)) ;
}

SpeedArbiter::Vote::Vote(float speed, int pwm)
   : m_speed(speed), m_pwm(pwm)
{}

//-------------------------- MOTOR COMMANDS -----------------------------

// According to Rosenblatt, to issue an appropriate motor command, the
// DAMN speed arbiter simply picks the minimum speed that will satisfy
// all the behaviours.
//
// However, for Robolocust, we prefer to take the behaviour priorities
// into account. Otherwise, for example, the extricate behaviour may
// never get to do its thing if the emergency_stop behaviour is always
// voting for a full stop. By issuing the drive commands of the highest
// priority behaviour, we ensure that low priority behaviours issuing
// lower speed commands don't override the directives of higher priority
// behaviours voting for higher speeds.
void SpeedArbiter::motor_cmd(const Arbiter::Votes& votes, Robot* robot)
{
   Arbiter::Votes::const_iterator max_priority =
      std::max_element(votes.begin(), votes.end(), compare_priorities(this)) ;

   const vote_data* D = *max_priority ;
   const Vote* V      = dynamic_cast<Vote*>(D->vote) ;
   //LERROR("vote: %-15s %10.3f [%5.2f %4d]",
          //D->behavior_name.c_str(), D->vote_time, V->speed(), V->pwm()) ;

   UpdateLock::begin_write() ;
#ifdef LOBOT_PRINT_VOTE_TO_MOTOR_DELAY
      LERROR("%-15s vote-to-motor delay = %5lld ms",
             D->behavior_name.c_str(), current_time() - D->vote_time) ;
#endif
      robot->drive(V->speed(), V->pwm()) ;
   UpdateLock::end_write() ;
}

// Helper function object to compare votes based on the priorities of the
// behaviours that cast them.
SpeedArbiter::compare_priorities::compare_priorities(const SpeedArbiter* A)
   : arbiter(A)
{}

bool
SpeedArbiter::compare_priorities::
operator()(const vote_data* a, const vote_data* b) const
{
   return arbiter->priority(a->behavior_name)
        < arbiter->priority(b->behavior_name) ;
}

//----------------------- TURN ARBITER CLEAN-UP -------------------------

SpeedArbiter::~SpeedArbiter(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
