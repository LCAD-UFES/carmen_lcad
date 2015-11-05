/**
   \file  Robots/LoBot/io/LoRobot.H
   \brief Abstract interface for the robot's sensorimotor system.

   This file defines an abstract base class that provides a high-level
   API for moving and steering the robot. This class also provides a
   Sensor object that holds the current values of the robot's sensors.
   Communication with the robot platform is achieved over a serial port.

   Different derived classes, by implementing the inner workings of this
   API, can be used to command different robot platforms and keep track
   of their sensors (e.g., a hacked R/C car or a Roomba).
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoRobot.H $
// $Id: LoRobot.H 13782 2010-08-12 18:21:14Z mviswana $
//

#ifndef LOBOT_ROBOT_DOT_H
#define LOBOT_ROBOT_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoSerial.H"
#include "Robots/LoBot/thread/LoMutex.H"
#include "Robots/LoBot/util/LoBits.H"

// INVT model manager stuff
#include "Component/ModelManager.H"

// Standard C++ headers
#include <vector>
#include <utility>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Robot
   \brief High-level API for driving and steering the robot and keeping
   track of its sensors.

   The Robolocust controller may run on different kinds of robots (e.g.,
   a hacked R/C car or a Roomba). All these robots are expected to be
   driven and queried for their sensors using a common interface as
   specified by this abstract base class. Derived classes will implement
   the actual innards of the interface for the different robots they are
   written for. These derived classes will be instantiated using a
   polymorphic factory and a config file setting.
*/
class Robot {
protected:
   /// The low-level serial device interface.
   ///
   /// DEVNOTE: We need to qualify the Serial class name with the
   /// namespace because LoSerial.H may use either libserial's SerialPort
   /// class or INVT's Serial class. If it is using INVT's serial
   /// support, then that class name will clash with the Robolocust
   /// Serial class encapsulating serial communications.
   lobot::Serial m_serial ;

public:
   /// Client modules interact with the physical robot platform via an
   /// instance of this (the Robot) class, which provides a common API
   /// for controlling the robot's motors and for reading its current
   /// sensor values.
   ///
   /// The robot's sensors are encapsulated using the Sensors inner class
   /// defined below. So as to maintain a common interface for all
   /// different robot platforms, this inner class needs to include
   /// fields for all the different sensors across all platforms. Sensors
   /// not relevant to the platform currently in use will be zeroed out.
   /// Clients should check the current platform before using those
   /// fields.
   ///
   /// For example, if the Roomba platform is being used, then the PWM
   /// and RPM sensor fields will not be relevant. Similarly, if the
   /// hacked R/C car is being used, then the cliff and wheel drop
   /// sensors will not make any sense.
   ///
   /// Thus, clients should determine which robot platform is being used
   /// before reading and using the sensor values stored in this class.
   /// This can be done by looking up the "platform" setting in the
   /// "robot" section of the lobot config file. Alternatively, we can
   /// write separate behaviours for different platforms and configure
   /// lobot to start the right set of behaviours depending on the robot
   /// platform being used.
   ///
   /// DEVNOTE: This inner class is a do-it-all interface and represents
   /// a design choke point. Every time lobot is extended to support
   /// another robot platform, this beast here will have to be hacked to
   /// include yet more fields to hold the sensor values for the new
   /// platform.
   ///
   /// Another way to implement this functionality might have been to
   /// implement a SensorBase class here and have every class derived
   /// from lobot::Robot provide its own Sensor class/struct containing
   /// only the relevant sensor fields. But because each robot platform
   /// can be quite different from every other, the SensorBase class
   /// defined here would be practically useless as a uniting interface
   /// and clients would have to use dynamic casts and/or RTTI to get the
   /// right type depending on the robot platform in use.
   ///
   /// The approach outlined above does not seem to be any more palatable
   /// than having this do-it-all class here. In fact, from clients'
   /// perspectives, this do-it-all interface may actually be nicer than
   /// having to know different types and cast as required.
   ///
   /// One other possibility is to not really have a statically typed
   /// class at all. Instead, we could use a "sensors database" that maps
   /// sensor names to values with both keys and values being strings
   /// (like the lobot configuration database). The Sensors API
   /// implemented here would then simply be map look-ups with return
   /// values being converted from strings to ints, floats, chars,
   /// whatever using a bunch of template functions (again, much as is
   /// done in the lobot configuration database).
   ///
   /// This way, clients would get a uniform interface to any robot's
   /// sensors. And the sensors interface defined here would simply be a
   /// key-value database that wouldn't need to be hacked each time the
   /// system is extended to support a new robot platform. Furthermore,
   /// each lobot::Robot derived class will have the ability/freedom to
   /// define any kind of sensor type without having to worry about
   /// conforming to some predetermined set of sensor types that can
   /// exist.
   ///
   /// Unfortunately, at this time (circa mid-Jan 2010), the "right" way
   /// to do it as described above will take more time than is available.
   /// Moreover, since development focus has now shifted to the Roomba
   /// (with the hacked R/C car platform being abandoned due to far too
   /// many hardware instabilities), the do-it-all approach is deemed an
   /// acceptable compromise. Eventually, when we get around to seriously
   /// adding support in this robot control framework for different
   /// platforms, the key-value database approach would be the way to go.
   class Sensors {
      /// Since this class is used to keep track of the "current" sensor
      /// values and many things happen asynchronously in the lobot
      /// controller, it is useful to know when the sensor readings were
      /// made.
      long long m_time_stamp ;

      /// Regardless of the robot platform, all classes derived from
      /// lobot::Robot will have to compute and update the current speed
      /// and heading because the speed and turn arbiters and behaviours
      /// all work in terms of those "fundamental" values.
      ///
      /// NOTE: The robot platform may not actually have sensors that
      /// directly report the current speed and heading. However, since
      /// all robot platforms are required to compute and update these
      /// values, it makes sense to have these fields in this class.
      /// Thus, conceptually, clients can work as if each supported robot
      /// platform has speed and heading sensors regardless of whether
      /// such things physically exist or not.
      float m_speed, m_heading ;

      /// The current values of the low-level motor duty cycles. These
      /// PWM values are sent to and received from the Propeller I/O
      /// board.
      ///
      /// As can be told by mention of the Propeller: these sensors are
      /// related to the hacked R/C car platform.
      int m_motor_pwm, m_servo_pwm ;

      /// The RPM sensor's value. This sensor is also applicable only to
      /// the hacked R/C car platform.
      float m_rpm ;

      /// The Roomba's bump and wheel drop sensors.
      ///
      /// NOTE: The front bump sensors are built into the iRobot Create
      /// platform. lobot, however, sports an additional bumper assembly
      /// on its rear.
      int m_bump, m_wheel_drops ;
      enum { // bit masks for above fields
         BUMP_REAR_LEFT  = 8,
         BUMP_REAR_RIGHT = 4,
         BUMP_LEFT  = 2,
         BUMP_RIGHT = 1,

         WHEEL_DROP_LEFT   = 4,
         WHEEL_DROP_RIGHT  = 2,
         WHEEL_DROP_CASTER = 1,
      } ;

      /// The Roomba's wall sensors and signal strength.
      int m_walls, m_wall_signal ;
      enum { // bit masks for above fields
         VIRTUAL_WALL = 2,
         WALL         = 1,
      } ;

      /// The Roomba's cliff sensors and signal strengths.
      int m_cliff, m_cliff_signals[4] ;
      enum { // bit masks and indices for above fields
         CLIFF_LEFT  = 8,
         CLIFF_RIGHT = 4,
         CLIFF_FRONT_LEFT  = 2,
         CLIFF_FRONT_RIGHT = 1,

         LEFT_SIGNAL = 0,
         RIGHT_SIGNAL,
         FRONT_LEFT_SIGNAL,
         FRONT_RIGHT_SIGNAL,
      } ;

      /// The Roomba's infrared byte, distance and angle measurements,
      /// and battery charge.
      int m_infrared, m_distance, m_angle, m_battery_charge ;

      /// Flag indicating whether the encoder info returned by Roomba
      /// contains result from most recent SPIN (i.e., in-place turn)
      /// command.
      bool m_spin_flag ;

      /// The Roomba's most recently requested drive speed and turn
      /// radius.
      int m_requested_speed, m_requested_radius ;

      /// Private default constructor because we only want the Robot
      /// class to be able to instantiate the Sensors class.
      Sensors() ;
      friend class Robot ;

   public:
      /// Public copy constructor; it is okay for client modules to make
      /// copies of the sensor data (but only the Robot class can create
      /// an instance from scratch).
      Sensors(const Sensors&) ;

      /// Accessors for the different sensor values.
      //@{
      long long time_stamp() const {return m_time_stamp ;}
      float speed()   const {return m_speed ;}
      float heading() const {return m_heading ;}

      int   motor_pwm() const {return m_motor_pwm ;}
      int   servo_pwm() const {return m_servo_pwm ;}
      float rpm()       const {return m_rpm ;}

      bool bump_left()  const {return m_bump & BUMP_LEFT  ;}
      bool bump_right() const {return m_bump & BUMP_RIGHT ;}
      bool bump_both()  const {return bump_left() && bump_right() ;}
      bool bump_front() const {return bump_left() || bump_right() ;}
      bool bump_rear_left()  const {return m_bump & BUMP_REAR_LEFT  ;}
      bool bump_rear_right() const {return m_bump & BUMP_REAR_RIGHT ;}
      bool bump_rear_both()  const {
         return bump_rear_left() && bump_rear_right() ;
      }
      bool bump_rear() const {return bump_rear_left() || bump_rear_right() ;}
      bool bump()      const {return bump_front() || bump_rear() ;}

      bool wheel_drop_left()   const{return m_wheel_drops & WHEEL_DROP_LEFT  ;}
      bool wheel_drop_right()  const{return m_wheel_drops & WHEEL_DROP_RIGHT ;}
      bool wheel_drop_caster() const{return m_wheel_drops & WHEEL_DROP_CASTER;}
      bool wheel_drop_all()    const{
         return wheel_drop_left() && wheel_drop_right() && wheel_drop_caster();
      }
      bool wheel_drop() const {
         return wheel_drop_left() || wheel_drop_right() || wheel_drop_caster();
      }

      bool wall()         const {return m_walls & WALL ;}
      bool virtual_wall() const {return m_walls & VIRTUAL_WALL ;}
      int  wall_signal()  const {return m_wall_signal ;}
      bool wall(int threshold) const {
         return wall() && wall_signal() >= threshold ;
      }

      bool cliff_left()  const {return m_cliff & CLIFF_LEFT  ;}
      bool cliff_right() const {return m_cliff & CLIFF_RIGHT ;}
      bool cliff_front_left()  const {return m_cliff & CLIFF_FRONT_LEFT  ;}
      bool cliff_front_right() const {return m_cliff & CLIFF_FRONT_RIGHT ;}
      bool cliff_all() const {
         return cliff_left()       && cliff_right()
             && cliff_front_left() && cliff_front_right() ;
      }
      bool cliff() const {
         return cliff_left()       || cliff_right()
             || cliff_front_left() || cliff_front_right() ;
      }
      int cliff_signal_left() const  {return m_cliff_signals[LEFT_SIGNAL]  ;}
      int cliff_signal_right() const {return m_cliff_signals[RIGHT_SIGNAL] ;}
      int cliff_signal_front_left()  const {
         return m_cliff_signals[FRONT_LEFT_SIGNAL] ;
      }
      int cliff_signal_front_right() const {
         return m_cliff_signals[FRONT_RIGHT_SIGNAL] ;
      }
      inline bool cliff(int threshold) const ;

      int  infrared() const {return m_infrared ;}
      int  distance() const {return m_distance ;}
      int  angle()    const {return m_angle    ;}
      bool spin()     const {return m_spin_flag;}

      int battery_charge()   const {return m_battery_charge   ;}
      int requested_speed()  const {return m_requested_speed  ;}
      int requested_radius() const {return m_requested_radius ;}
      //@}
   } ;

protected:
   /// The robot's current sensor values are all maintained in an
   /// instance of the Sensors inner class defined above. Derived classes
   /// are expected to update this object and client modules may retrieve
   /// a reference to it to examine the robot's current state.
   Sensors m_sensors ;

   /// Protected API for setting sensor values because only derived
   /// classes (i.e., concrete robot platforms) should be allowed to
   /// update the robot's sensor state.
   //@{
   void time_stamp(long long ts) {m_sensors.m_time_stamp = ts ;}

   void speed(float s)   {m_sensors.m_speed   = s ;}
   void heading(float h) {m_sensors.m_heading = h ;}

   void motor_pwm(int p) {m_sensors.m_motor_pwm = p ;}
   void servo_pwm(int p) {m_sensors.m_servo_pwm = p ;}
   void rpm(float r)     {m_sensors.m_rpm = r ;}

   void bump_left(bool b) {
      m_sensors.m_bump = set_bit(m_sensors.m_bump, Sensors::BUMP_LEFT, b) ;
   }
   void bump_right(bool b) {
      m_sensors.m_bump = set_bit(m_sensors.m_bump, Sensors::BUMP_RIGHT, b) ;
   }
   void bump_rear_left(bool b) {
      m_sensors.m_bump = set_bit(m_sensors.m_bump, Sensors::BUMP_REAR_LEFT, b);
   }
   void bump_rear_right(bool b) {
      m_sensors.m_bump = set_bit(m_sensors.m_bump, Sensors::BUMP_REAR_RIGHT,b);
   }

   void wheel_drop_left(bool b) {
      m_sensors.m_wheel_drops =
         set_bit(m_sensors.m_wheel_drops, Sensors::WHEEL_DROP_LEFT, b) ;
   }
   void wheel_drop_right(bool b) {
      m_sensors.m_wheel_drops =
         set_bit(m_sensors.m_wheel_drops, Sensors::WHEEL_DROP_RIGHT, b) ;
   }
   void wheel_drop_caster(bool b) {
      m_sensors.m_wheel_drops =
         set_bit(m_sensors.m_wheel_drops, Sensors::WHEEL_DROP_CASTER, b) ;
   }

   void wall(bool b) {
      m_sensors.m_walls = set_bit(m_sensors.m_walls, Sensors::WALL, b) ;
   }
   void virtual_wall(bool b) {
      m_sensors.m_walls = set_bit(m_sensors.m_walls, Sensors::VIRTUAL_WALL, b);
   }
   void wall_signal(int s) {m_sensors.m_wall_signal = s ;}

   void cliff_left(bool b) {
      m_sensors.m_cliff = set_bit(m_sensors.m_cliff, Sensors::CLIFF_LEFT, b) ;
   }
   void cliff_right(bool b) {
      m_sensors.m_cliff = set_bit(m_sensors.m_cliff, Sensors::CLIFF_RIGHT, b) ;
   }
   void cliff_front_left(bool b) {
      m_sensors.m_cliff =
         set_bit(m_sensors.m_cliff, Sensors::CLIFF_FRONT_LEFT, b) ;
   }
   void cliff_front_right(bool b) {
      m_sensors.m_cliff =
         set_bit(m_sensors.m_cliff, Sensors::CLIFF_FRONT_RIGHT, b) ;
   }

   void cliff_left_signal (int s) {
      m_sensors.m_cliff_signals[Sensors::LEFT_SIGNAL]  = s ;
   }
   void cliff_right_signal(int s) {
      m_sensors.m_cliff_signals[Sensors::RIGHT_SIGNAL] = s ;
   }
   void cliff_front_left_signal (int s) {
      m_sensors.m_cliff_signals[Sensors::FRONT_LEFT_SIGNAL]  = s ;
   }
   void cliff_front_right_signal(int s) {
      m_sensors.m_cliff_signals[Sensors::FRONT_RIGHT_SIGNAL] = s ;
   }

   void infrared(int i)   {m_sensors.m_infrared  = i ;}
   void distance(int d)   {m_sensors.m_distance  = d ;}
   void angle(int a)      {m_sensors.m_angle     = a ;}
   void spin_flag(bool f) {m_sensors.m_spin_flag = f ;}

   void battery_charge(int c)   {m_sensors.m_battery_charge   = c ;}
   void requested_speed (int s) {m_sensors.m_requested_speed  = s ;}
   void requested_radius(int r) {m_sensors.m_requested_radius = r ;}
   //@}

public:
   /// Usually, when a behaviour or other client module wants to know the
   /// current sensor state, it can simply query the robot interface
   /// object (via lobot::App) for the desired data. However, in certain
   /// situations, a behaviour might want to be informed about low-level
   /// sensor updates as soon as they come in. For example, if a
   /// behaviour is performing odometry-based localization, it cannot
   /// afford to miss low-level sensor updates by querying in its action
   /// method because it could well get incorrect odometric info by the
   /// time the behaviour's main loop's next iteration kicks in.
   ///
   /// The robot interface object, therefore, maintains a list of
   /// callback functions, which it calls one-by-one right after the main
   /// thread updates the robot's sensor state. These types define the
   /// sensor hook functions.
   ///
   /// NOTE: The sensor hooks are executed in the context of the main
   /// thread. So, clients should take appropriate measures to protect
   /// any shared data structures that are used in the hook functions.
   /// Furthermore, to avoid delaying the main thread, clients should
   /// keep the processing within these hook functions to a minimum.
   //@{
   typedef void (*SensorUpdateCB)(const Sensors&, unsigned long client_data) ;
   typedef std::pair<SensorUpdateCB, unsigned long> SensorHook ;
   //@}

private:
   /// This data structure holds the hook functions.
   std::vector<SensorHook> m_sensor_hooks ;

   /// Because the list of sensor hooks can be accessed in multiple
   /// threads, we need a mutex to synchronize simultaneous accesses.
   Mutex m_sensor_hooks_mutex ;

public:
   /// Client modules can retrieve the current sensor values by using
   /// this API.
   const Sensors& sensors() const {return m_sensors ;}

   /// Retrieving the robot's current speed and heading.
   //@{
   float current_speed()   const {return m_sensors.speed()   ;}
   float current_heading() const {return m_sensors.heading() ;}
   //@}

   /// Adding a sensor hook so as to be informed about low-level sensor
   /// updates as soon as they happen.
   void add_hook(const SensorHook&) ;

protected:
   /// A protected constructor because motor classes are instantiated via
   /// a factory and a config file setting rather than directly by client
   /// modules.
   Robot(const ModelManager&, const std::string& device, int baud_rate) ;

public:
   /// Clients must issue drive commands using both a speed expressed in
   /// m/s and a PWM value.
   ///
   /// The reason we require clients to supply both is because some robot
   /// platforms will support a speed command whereas others only provide
   /// for PWM commands. Robot platforms that support both can choose one
   /// type of command over the other based on config file settings,
   /// sensor readings, etc. Either ways, this decision is up to
   /// individual derived classes.
   ///
   /// DEVNOTE: Since this method will change the internal state of the
   /// motor system (which is shared by many behaviours/modules), clients
   /// should protect calls to it using lobot::UpdateLock.
   virtual void drive(float speed, int pwm) = 0 ;

   /// Command the robot to steer towards the specified direction
   /// (expressed in degrees). Zero degrees is for driving straight
   /// ahead; positive (ccw) angles are for heading left and negative
   /// (cw) angles for heading right.
   ///
   /// DEVNOTE: Since this method will change the internal state of the
   /// motor system (which is shared by many behaviours/modules), clients
   /// should protect calls to it using lobot::UpdateLock.
   virtual void turn(float direction) = 0 ;

   /// Command the robot to spin in-place by the specified angle
   /// (expressed in degrees). Positive angles result in counterclockwise
   /// in-place turns and negative angles in clockwise in-place turns.
   ///
   /// DEVNOTE: Since this method will change the internal state of the
   /// motor system (which is shared by many behaviours/modules), clients
   /// should protect calls to it using lobot::UpdateLock.
   virtual void spin(float angle) = 0 ;

   /// This method is meant to be called by the Robolocust main thread as
   /// part of the sensorimotor update loop. It should not be called by
   /// any other modules. The idea here is that the main thread updates
   /// all the sensors and motor systems while all other behaviours and
   /// modules simply read the latest measurements or motor states.
   ///
   /// DEVNOTE: Since this method will change the internal state of the
   /// motor system (which is shared by many behaviours/modules), clients
   /// should protect calls to it using lobot::UpdateLock.
   void update() ;

private:
   /// All concrete robot interface objects must define this method for
   /// updating the low-level sensors. This method is invoked by
   /// lobot::Robot::update(). Once this method is done, the base class
   /// will trigger the sensor hooks.
   ///
   /// The base class expects this method to return a flag indicating
   /// whether or not sensors were actually updated and, thus, whether
   /// or not the sensor hooks need to be triggered. If there truly is a
   /// new sensor packet sent by the low-level controller, this method
   /// should return true; false otherwise.
   ///
   /// NOTE: It is crucial that derived classes encapsulating concrete
   /// robot platforms correctly handle the return value of this
   /// function. Otherwise, it is possible for client sensor hooks to be
   /// called multiple times with the same packet, which can lead to odd
   /// behaviour in client modules (e.g., a localization module may end
   /// up applying the same odometry over and over again, resulting in a
   /// horrendously mislocalized robot).
   virtual bool update_sensors() = 0 ;

public:
   /// Query the motor subsystem to see if the robot is currently moving
   /// or stationary.
   ///
   /// NOTE: Derived classes may override this method if they wish to
   /// implement it in a different way from the default, which is to
   /// simply check if the speed is zero.
   virtual bool stopped() const ;

   /// A convenience function for stopping the robot and straightening
   /// its wheels.
   ///
   /// NOTE: The default implementation of this function simply issues
   /// drive and turn commands with zero as the parameters. However,
   /// derived classes may want to override this implementation if their
   /// particular robot platforms provide different or better ways to
   /// turn the motor system off.
   virtual void off() ;

   /// Clean-up.
   virtual ~Robot() ;
} ;

//---------------------- INLINE MEMBER FUNCTIONS ------------------------

// Return true if a cliff is detected and that cliff's signal strength
// exceeds the supplied threshold.
inline bool Robot::Sensors::cliff(int threshold) const
{
   return (cliff_left()  && cliff_signal_left()  >= threshold)
       || (cliff_right() && cliff_signal_right() >= threshold)
       || (cliff_front_left()  && cliff_signal_front_left()  >= threshold)
       || (cliff_front_right() && cliff_signal_front_right() >= threshold) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
