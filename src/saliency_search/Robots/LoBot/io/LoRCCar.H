/**
   \file  Robots/LoBot/io/LoRCCar.H
   \brief Interface object for driving the robot built on top of a hacked
   R/C car and retrieving its sensor values.

   This file defines a class that implements the motor driving interface
   for lobot as built on top of the hacked R/C car. This robot uses a
   mini-ITX computer running Debian as the computing platform and a
   Propeller I/O board connected to the computer via a USB serial
   connection for sending drive and turn commands to the robot's motors.
   Drive commands are sent to a Sabertooth motor driver and turn commands
   to a steering servo.

   Additionally, the robot is equipped with a Hall effect sensor mounted
   on its right wheel, which is used to gauge the robot's speed. This
   object also takes care of retrieving the RPM value returned by this
   sensor and converting that to a speed reading.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoRCCar.H $
// $Id: LoRCCar.H 13770 2010-08-08 06:49:50Z mviswana $
//

#ifndef LOBOT_RC_CAR_DOT_H
#define LOBOT_RC_CAR_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoRobot.H"

#include "Robots/LoBot/misc/LoPID.H"
#include "Robots/LoBot/misc/factory.hh"
#include "Robots/LoBot/misc/singleton.hh"
#include "Robots/LoBot/misc/wma.hh"
#include "Robots/LoBot/util/triple.hh"

// INVT model manager stuff
#include "Component/ModelManager.H"

// Standard C++ headers
#include <string>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::RCCar
   \brief High-level API for driving and steering the R/C car based
   robot and retrieving the value of its RPM sensor.

   The robot's motors are controlled by a motor driver that is in turn
   controlled by a Propeller-based I/O board. This board connects to the
   Robolocust computer (the mini-ITX Debian box) over a USB port. The
   lobot::RCcar class implements the motor API defined by lobot::Robot by
   sending off all the drive and turn commands to the Propeller.

   Additionally, the Propeller board also interfaces with a Hall effect
   sensor and computes the RPM of the robot's front right wheel.
*/
class RCCar : public Robot {
   // Handy type to have around in a derived class
   typedef Robot base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<RCCar, base, ModelManager> ;
   typedef register_factory<RCCar, base, ModelManager> my_factory ;
   static  my_factory register_me ;

   /// The hacked R/C car's motor subsytem uses a PID controller to
   /// achieve the speed specified by its clients.
   PID<float> m_speed_pid ;

   /// The speed PID controller doesn't work as well as it could due to
   /// the input speed commands being excessively noisy. Since these
   /// inputs are computed on the basis of the RPM sensor readings, we
   /// need to clean those up. We do so with a weighted moving average
   /// (which acts as a kind of low pass filter).
   wma<float> m_rpm_filter ;

   /// Private constructor because the interface object for a robot's
   /// motor subsystem is created using a factory.
   ///
   /// Generally, the Robolocust project prefers the libserial C++
   /// library for serial communications to the serial support provided
   /// by INVT. However, in case libserial is not available, we fall back
   /// to INVT, which is why the constructor needs the application's INVT
   /// ModelManager (because INVT's Serial class requires it).
   RCCar(const ModelManager&) ;

   /// Clients must issue drive commands using both a speed expressed in
   /// m/s and a PWM value. A flag in the config file specifies which one
   /// is actually used.
   ///
   /// The reason we do this is because drive commands based on speeds
   /// rely on the RPM sensor attached to the front right wheel of the
   /// robot. This sensor is extremely noisy and quite unreliable.
   /// Therefore, we allow users to bypass the RPM sensor altogether if
   /// they so wish and work directly in terms of motor PWM values.
   /// Consequently, the public interface to the motor drive commands
   /// require client modules to supply both parameters. Then,
   /// internally, the lobot::Motor class uses the bypass_rpm_sensor flag
   /// to decide which parameter to actually use.
   void drive(float speed, int pwm) ;

   /// Command the motors to drive at the specified speed (expressed in
   /// m/s). Positive speeds indicate forward motion, negative speeds are
   /// for reversing and zero indicates coming to a full stop.
   void drive(float speed) ;

   /// Command the motors to drive at the specified motor PWM value.
   /// Positive quantities indicate forward motion, negative ones are for
   /// reversing and zero indicates coming to a full stop.
   void drive(int pwm) ;

   /// Command the motors to turn towards the specified direction
   /// (expressed in degrees). Zero degrees is for driving straight
   /// ahead; positive (ccw) angles are for heading left and negative
   /// (cw) angles for heading right.
   void turn(float direction) ;

   /// Command the robot to spin in-place by the specified angle
   /// (expressed in degrees). Positive angles result in ccw in-place
   /// turns while negative angles are for cw in-place turns.
   void spin(float angle) ;

   /// Stop robot and straighten wheels.
   void off() ;

   /// Get the current speed and heading.
   bool update_sensors() ;

   /// Check if the robot is moving or stationary.
   bool stopped() const ;

   /// Low-level motor commands.
   //@{
   enum {
      // Motor commands
      FORWARD  = 100,
      REVERSE  = 101,
      STOP     = 102,
      LEFT     = 110,
      RIGHT    = 111,
      STRAIGHT = 112,
      OFF      = 120,

      // Propeller status commands
      GET_MOTOR_DIR = 130,
      GET_MOTOR_PWM = 131,
      GET_SERVO_DIR = 132,
      GET_SERVO_PWM = 133,
      GET_RPM       = 134,
   } ;
   void send_propeller(int cmd) ;
   void send_propeller(int cmd, int param) ;
   int  recv_propeller(int cmd, int n = 1) ;
   //@}

   /// Clean-up.
  ~RCCar() ;

   /// This inner class encapsulates various parameters that can be used
   /// to tweak different aspects of the motor interface.
   class Params : public singleton<Params> {
      /// The robot's motors are controlled by sending commands over a
      /// serial port. This setting specifies the serial port device that
      /// should be used.
      std::string m_device ;

      /// This setting specifies the serial port communication rate.
      int m_baud_rate ;

      /// The robot's turn arbiter issues steering commands in terms of
      /// turn angles. The low-level motor interface needs to convert
      /// these directions into appropriate steering servo PWM values. In
      /// order to effect the transformation from turn angle to servo
      /// PWM, we need to know the max angle the turn arbiter will issue.
      int m_turn_max ;

      /// The diameter of the robot's wheels. This is used to compute the
      /// current speed at which the robot is moving.
      ///
      /// This parameter is expected to be specified in millimeters.
      float m_wheel_diameter ;

      /// To convert RPM values to speeds, we multiply the RPM by pi
      /// times the wheel diameter and divide by 60 to get the resulting
      /// speed in millimeters per second. Dividing by 1000 gives us m/s.
      ///
      /// This parameter is used to keep track of the above calculation.
      /// It is not a configuration setting meant for end-user tweaking.
      /// Rather, it is a handy precomputed factor kept around to avoid
      /// repeating the same calculation over and over.
      float m_rpm_speed_factor ;

      /// The higher layers of the Robolocust controller specify drive
      /// commands in terms of a speed expressed in m/s. The low-level
      /// motor system, however, needs to be commanded in terms of a PWM
      /// duty cycle. To convert speed commands to appropriate PWM
      /// values, we first convert the target speed set by some
      /// high-level behaviour/module into RPM and then just multiply the
      /// result with some suitable conversion factor.
      ///
      /// For maximum flexibility, we let users specify an appropriate
      /// conversion factor.
      float m_rpm_pwm_factor ;

      /// Since the RPM sensor is quite noisy, we need to filter its
      /// readings. We do so using a weighted moving average (a sort of
      /// low pass filter). This setting specifies the number of previous
      /// readings that should be considered for computing the
      /// above-mentioned weighted moving average.
      ///
      /// NOTE: Although inadvisable, the RPM filter can be turned off by
      /// setting this config value to one.
      int m_rpm_filter_size ;

      /// Despite the filteration applied to it, the RPM sensor can still
      /// return extremely bogus readings! Sometimes, we may simply want
      /// to turn that dang thing off and not bother with it at all. If
      /// this flag is set, the motor system will be commanded in terms
      /// of PWM values instead of drive velocities expressed in m/s.
      bool m_bypass_rpm_sensor ;

      /// The motor subsystem uses a PID controller to achieve and
      /// maintain the target speed commanded by high-level behaviours
      /// and modules. This setting specifies the PID gains for the speed
      /// controller.
      triple<float, float, float> m_speed_gains ;

      /// This parameter converts steering directions sent by the turn
      /// arbiter to corresponding PWM values to be sent to the steering
      /// servo (and vice versa).
      float m_steering_pwm_factor ;

      /// Private constructor because this is a singleton.
      Params() ;

      // Boilerplate code to make generic singleton design pattern work
      friend class singleton<Params> ;

   public:
      /// Accessing the various parameters.
      //@{
      static const std::string& device() {return instance().m_device ;}
      static int   baud_rate()        {return instance().m_baud_rate ;}
      static int   turn_max()         {return instance().m_turn_max ;}
      static float wheel_diameter()   {return instance().m_wheel_diameter ;}
      static float rpm_speed_factor() {return instance().m_rpm_speed_factor ;}
      static float rpm_pwm_factor()   {return instance().m_rpm_pwm_factor ;}
      static bool  bypass_rpm_sensor(){return instance().m_bypass_rpm_sensor ;}
      static int   rpm_filter_size()  {return instance().m_rpm_filter_size ;}
      static const triple<float, float, float>& speed_gains() {
         return instance().m_speed_gains ;
      }
      static float steering_pwm_factor() {
         return instance().m_steering_pwm_factor ;
      }
      //@}

      /// Clean-up.
      ~Params() ;
   } ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
