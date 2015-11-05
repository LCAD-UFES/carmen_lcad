/**
   \file  Robots/LoBot/io/LoRCCar.C
   \brief This file defines the non-inline member functions of the
   lobot::RCCar class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoRCCar.C $
// $Id: LoRCCar.C 13911 2010-09-10 03:14:57Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoRCCar.H"
#include "Robots/LoBot/control/LoTurnArbiter.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/util/LoTime.H"

// INVT utilities
#include "Util/log.H"

//------------------------------ MACROS ---------------------------------

// Sometimes, during development, the motors may not be available (e.g.,
// when writing code at home instead of at iLab). Or, we may simply not
// want to use the actual motors (e.g., we're testing some behaviour and
// only want to see/visualize its votes without driving the motors).
//
// In these situations, it can be useful to have a convenient dummy motor
// system available. The following symbol can be used for the
// above-mentioned purpose.
//
// Un/comment following line when motors are un/available.
//#define LOBOT_MOTOR_DEVMODE 1

// The motor system's update method prints the current speed, heading,
// etc. Sometimes, we may want to suppress this printing. For that,
// uncomment the following line.
//#define LOBOT_MOTOR_NO_PRINT 1

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

RCCar::RCCar(const ModelManager& mgr)
   : base(mgr, Params::device(), Params::baud_rate()),
     m_speed_pid(Params::speed_gains()),
     m_rpm_filter(Params::rpm_filter_size())
{}

//-------------------------- MOTOR COMMANDS -----------------------------

void RCCar::drive(float speed, int pwm)
{
   if (Params::bypass_rpm_sensor())
      drive(pwm) ;
   else
      drive(speed) ;
}

void RCCar::drive(float speed)
{
   if (is_zero(speed))
   {
      send_propeller(STOP) ;
      m_speed_pid.reset() ;
      m_rpm_filter.reset() ;
      return ;
   }

   float speed_cmd = m_speed_pid.cmd(speed - current_speed()) ;
   float rpm_cmd   = speed_cmd/Params::rpm_speed_factor() ;
   int pwm = sensors().motor_pwm() + round(rpm_cmd * Params::rpm_pwm_factor());
   drive(pwm) ;
}

void RCCar::drive(int pwm)
{
   if (pwm < 0)
      send_propeller(REVERSE, clamp(-pwm, 10, 100) * 127/100) ;
   else if (pwm > 0)
      send_propeller(FORWARD, clamp( pwm, 10, 100) * 127/100) ;
   else
      send_propeller(STOP) ;
}

void RCCar::turn(float direction)
{
   if (direction < 0) // negative angle ==> turn right
      send_propeller(RIGHT, round(-direction * Params::steering_pwm_factor()));
   else if (direction > 0) // positive angle ==> turn left
      send_propeller(LEFT,  round( direction * Params::steering_pwm_factor()));
   else
      send_propeller(STRAIGHT) ;
}

void RCCar::spin(float)
{
   throw motor_error(IN_PLACE_TURNS_NOT_SUPPORTED) ;
}

void RCCar::off()
{
   //LERROR("switching off motors...") ;
   send_propeller(OFF) ;

   // DEVNOTE: After sending the OFF command to the Propeller, we also
   // seem to need to give the command some time to "sink in." For some
   // strange and unknown reason, without this "sinking in time," the
   // motors don't actually stop.
   //
   // We could simply sleep for a while to ensure that the Propeller gets
   // the OFF command and responds properly. However, instead of blithely
   // sleeping for some time, we can call the update function to retrieve
   // the low-level motor state and print that out. This not only effects
   // the delay necessary for the "sink-in time" explained above but also
   // provides the user with some feedback that the motors really have
   // gone off.
   update() ;
}

//--------------------------- MOTOR STATUS ------------------------------

bool RCCar::update_sensors()
{
   int M = ((recv_propeller(GET_MOTOR_DIR) == REVERSE) ? -1 : +1)
      * recv_propeller(GET_MOTOR_PWM) * 100/127 ;
   int S = ((recv_propeller(GET_SERVO_DIR) == RIGHT) ? -1 : +1)
      * recv_propeller(GET_SERVO_PWM) ;
   float R = recv_propeller(GET_RPM, 4)/1000.0f ; // Propeller returns rpm*1000

   motor_pwm(M) ;
   servo_pwm(S) ;
   rpm(R) ;
   m_rpm_filter.add(R) ;
   speed(sign(M) * m_rpm_filter.value() * Params::rpm_speed_factor()) ;
   heading(round(S/Params::steering_pwm_factor())) ;
   time_stamp(current_time()) ;

#ifndef LOBOT_MOTOR_NO_PRINT
   LERROR("drive = [%4d %8.2f %8.2f %7.2f]; turn = [%4d %6.1f]",
          M, R, m_rpm_filter.value(), current_speed(), S, current_heading()) ;
#endif
   return true ; // bogus! but then we're not using this class anymore...
}

bool RCCar::stopped() const
{
   return sensors().motor_pwm() == 0 ;
}

//--------------------- LOW-LEVEL MOTOR COMMANDS ------------------------

#ifdef LOBOT_MOTOR_DEVMODE

// Development mode dummy
class fake_propeller {
   int motor_dir, motor_pwm ;
   int servo_dir, servo_pwm ;

   enum {
      FORWARD_CMD = 100,
      REVERSE_CMD = 101,
      STOP_CMD    = 102,

      LEFT_CMD     = 110,
      RIGHT_CMD    = 111,
      STRAIGHT_CMD = 112,

      OFF_CMD = 120,

      GET_MOTOR_DIR = 130,
      GET_MOTOR_PWM = 131,
      GET_SERVO_DIR = 132,
      GET_SERVO_PWM = 133,
      GET_RPM       = 134,
   } ;

public:
   fake_propeller() ;
   void send(int cmd) ;
   void send(int cmd, int param) ;
   int  recv(int cmd) ;
} ;

fake_propeller::fake_propeller()
   : motor_dir(0), motor_pwm(0),
     servo_dir(0), servo_pwm(0)
{}

void fake_propeller::send(int cmd)
{
   switch (cmd)
   {
      case STOP_CMD:
         motor_dir = STOP_CMD ;
         motor_pwm = 0 ;
         break ;
      case STRAIGHT_CMD:
         servo_dir = STRAIGHT_CMD ;
         servo_pwm = 0 ;
         break ;
      case OFF_CMD:
         motor_dir = STOP_CMD ;
         motor_pwm = 0 ;
         servo_dir = STRAIGHT_CMD ;
         servo_pwm = 0 ;
         break ;
   }
}

void fake_propeller::send(int cmd, int param)
{
   switch (cmd)
   {
      case FORWARD_CMD:
         motor_dir = FORWARD_CMD ;
         motor_pwm = clamp(param, 0, 127) ;
         break ;
      case REVERSE_CMD:
         motor_dir = REVERSE_CMD ;
         motor_pwm = clamp(param, 0, 127) ;
         break ;
      case LEFT_CMD:
         servo_dir = LEFT_CMD ;
         servo_pwm = clamp(param, 0, 100) ;
         break ;
      case RIGHT_CMD:
         servo_dir = RIGHT_CMD ;
         servo_pwm = clamp(param, 0, 100) ;
         break ;
   }
}

int fake_propeller::recv(int cmd)
{
   switch (cmd)
   {
      case GET_MOTOR_DIR:
         return motor_dir ;
      case GET_MOTOR_PWM:
         return motor_pwm ;
      case GET_SERVO_DIR:
         return servo_dir ;
      case GET_SERVO_PWM:
         return servo_pwm ;
      case GET_RPM:
         return 0 ;
   }
   return -1 ;
}

static fake_propeller propeller ;

void RCCar::send_propeller(int cmd)
{
   propeller.send(cmd) ;
}

void RCCar::send_propeller(int cmd, int param)
{
   propeller.send(cmd, param) ;
}

int RCCar::recv_propeller(int cmd, int)
{
   return propeller.recv(cmd) ;
}

#else // the real Propeller interface

void RCCar::send_propeller(int cmd)
{
   char buf[] = {cmd} ;
   m_serial.write(buf, sizeof(buf)) ;
}

void RCCar::send_propeller(int cmd, int param)
{
   char buf[] = {cmd, param} ;
   m_serial.write(buf, sizeof(buf)) ;
}

// The recv_propeller() function will cause a strict aliasing warning
// because of the way it casts the buf variable. We turn off the warning
// to allow INVT builds to succeed (because it uses -Werror by default).
//
// DEVNOTE: In general, this is not a good thing to do. Ideally, we
// should fix the return statement that performs the weird cast that
// causes the compiler warning. However, in this case, we really do want
// that cast the way it is.
//
// Moreover, this particular class, viz., the interface for the older R/C
// car based robot platform is not really in use anymore (circa September
// 2010). Nonetheless, it is a useful class to have around, if for
// nothing else other than illustration as to how the Robolocust
// framework can be adapted to different robot platforms. Therefore, we
// let the function go unfixed and instead opt to turn off this
// particular warning so that builds succeed without issue.
//
// DEVNOTE 2: The diagnostic pragma for ignoring warnings may not be
// available on versions of GCC older than 4.3.0. It is important to
// perform this check; otherwise, GCC will issue a warning about the
// pragma directive and again cause the build to fail no thanks to INVT's
// use of the -Werror option.
//
// DEVNOTE 3: GCC version 4.1.2, which ships with Debian 4.0 (etch),
// seems not to care about strict aliasing and so does not complain when
// compiling recv_propeller(). However, this may not be so on other
// system with other versions of GCC. In that case, fixing the offending
// return statement in recv_propeller() may be the only way out of the
// quandary.
#if __GNUC__ >= 4 && __GNUC_MINOR__ >= 3
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif

int RCCar::recv_propeller(int cmd, int n)
{
   send_propeller(cmd) ;

   const int MAX_TRIES = 10 ;
   int tries = 0 ;
   for(;;)
   {
      usleep(50000) ;

      char buf[n + 1] ;
      std::fill_n(buf, n + 1, 0) ;

      if (m_serial.read(buf, n) >= n) {
         std::reverse(buf, buf + n) ;
         return *reinterpret_cast<int*>(buf) ;
      }

      if (++tries <= MAX_TRIES)
         continue ;
      throw motor_error(MOTOR_READ_FAILURE) ;
   }
   return 0 ; // just to keep the compiler happy
}

#endif // LOBOT_MOTOR_DEVMODE

//----------------------------- CLEAN-UP --------------------------------

RCCar::~RCCar(){}

//-------------------------- KNOB TWIDDLING -----------------------------

// Parameters initialization
RCCar::Params::Params()
   : m_device(robot_conf<std::string>("serial_port", "/dev/ttyUSB0")),
     m_baud_rate(clamp(robot_conf("baud_rate", 38400), 50, 115200)),
     m_wheel_diameter(clamp(robot_conf("wheel_diameter", 139.70f),
                            10.0f, 1000.0f)),
     m_rpm_speed_factor(3.14159f * m_wheel_diameter/60000),
     m_rpm_pwm_factor(clamp(robot_conf("rpm_pwm_factor", 0.3f),
                            0.001f, 1.0f)),
     m_rpm_filter_size(clamp(robot_conf("rpm_filter_size", 10), 1, 100)),
     m_bypass_rpm_sensor(robot_conf("bypass_rpm_sensor", false)),
     m_speed_gains(get_conf("robot", "speed_gains",
                            make_triple(1.0f, 0.01f, 0.0f))),
     m_steering_pwm_factor(clamp(robot_conf("max_steering_pwm", 75.0f),
                                 50.0f, 90.0f)/TurnArbiter::turn_max())
{}

// Parameters clean-up
RCCar::Params::~Params(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

//-----------------------------------------------------------------------

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
