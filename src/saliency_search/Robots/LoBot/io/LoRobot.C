/**
   \file  Robots/LoBot/io/LoRobot.C
   \brief This file defines the non-inline member functions of the
   lobot::Robot class.
*/

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoRobot.C $
// $Id: LoRobot.C 13593 2010-06-21 23:26:06Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoRobot.H"
#include "Robots/LoBot/util/LoMath.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

Robot::
Robot(const ModelManager& mgr, const std::string& device, int baud_rate)
   : m_serial(mgr, device, baud_rate)
{}

Robot::Sensors::Sensors()
   : m_time_stamp(0),
     m_speed(0), m_heading(0),
     m_motor_pwm(0), m_servo_pwm(0), m_rpm(0),
     m_bump(0), m_wheel_drops(0),
     m_walls(0), m_wall_signal(0),
     m_cliff(0),
     m_infrared(0),
     m_distance(0), m_angle(0),
     m_battery_charge(0),
     m_requested_speed(0), m_requested_radius(0)
{
   m_cliff_signals[LEFT_SIGNAL]  = 0 ;
   m_cliff_signals[RIGHT_SIGNAL] = 0 ;
   m_cliff_signals[FRONT_LEFT_SIGNAL]  = 0 ;
   m_cliff_signals[FRONT_RIGHT_SIGNAL] = 0 ;
}

Robot::Sensors::Sensors(const Robot::Sensors& S)
   : m_time_stamp(S.m_time_stamp),
     m_speed(S.m_speed), m_heading(S.m_heading),
     m_motor_pwm(S.m_motor_pwm), m_servo_pwm(S.m_servo_pwm), m_rpm(S.m_rpm),
     m_bump(S.m_bump), m_wheel_drops(S.m_wheel_drops),
     m_walls(S.m_walls), m_wall_signal(S.m_wall_signal),
     m_cliff(S.m_cliff),
     m_infrared(S.m_infrared),
     m_distance(S.m_distance), m_angle(S.m_angle),
     m_battery_charge(S.m_battery_charge),
     m_requested_speed(S.m_requested_speed),
     m_requested_radius(S.m_requested_radius)
{
   m_cliff_signals[LEFT_SIGNAL]  = S.m_cliff_signals[LEFT_SIGNAL] ;
   m_cliff_signals[RIGHT_SIGNAL] = S.m_cliff_signals[RIGHT_SIGNAL] ;
   m_cliff_signals[FRONT_LEFT_SIGNAL]  = S.m_cliff_signals[FRONT_LEFT_SIGNAL] ;
   m_cliff_signals[FRONT_RIGHT_SIGNAL] = S.m_cliff_signals[FRONT_RIGHT_SIGNAL];
}

//-------------------------- SENSOR UPDATES -----------------------------

void Robot::add_hook(const Robot::SensorHook& H)
{
   AutoMutex M(m_sensor_hooks_mutex) ;
   m_sensor_hooks.push_back(H) ;
}

class trigger_hook {
   const Robot::Sensors& m_sensors ;
public:
   trigger_hook(const Robot::Sensors& S) : m_sensors(S) {}
   void operator()(const Robot::SensorHook& H) const {
      //LERROR("triggering sensor hook for packet @ %lld",
             //m_sensors.time_stamp()) ;
      H.first(m_sensors, H.second) ;
   }
} ;

void Robot::update()
{
   if (update_sensors()) {
      AutoMutex M(m_sensor_hooks_mutex) ;
      std::for_each(m_sensor_hooks.begin(), m_sensor_hooks.end(),
                    trigger_hook(m_sensors)) ;
   }
}

//-------------------------- MOTOR COMMANDS -----------------------------

void Robot::off()
{
   turn(0) ;
   drive(0, 0) ;
}

//------------------------ MOTOR STATE QUERIES --------------------------

bool Robot::stopped() const
{
   return is_zero(current_speed()) ;
}

//----------------------------- CLEAN-UP --------------------------------

Robot::~Robot(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

//-----------------------------------------------------------------------

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
