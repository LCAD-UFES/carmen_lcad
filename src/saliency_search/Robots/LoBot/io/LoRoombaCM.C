/**
   \file  Robots/LoBot/io/LoRoombaCM.C
   \brief This file defines the non-inline member functions of the
   lobot::RoombaCM class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoRoombaCM.C $
// $Id: LoRoombaCM.C 13783 2010-08-12 19:36:05Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoRoombaCM.H"
#include "Robots/LoBot/control/LoTurnArbiter.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"

#include "Robots/LoBot/thread/LoShutdown.H"
#include "Robots/LoBot/thread/LoPause.H"

#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/singleton.hh"

#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/util/LoBits.H"
#include "Robots/LoBot/util/LoTime.H"

#include "Robots/LoBot/irccm/LoOpenInterface.h"

// INVT utilities
#include "Util/log.H"

// Standard C++ headers
#include <algorithm>
#include <vector>
#include <utility>

// Standard C headers
#include <ctype.h>

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

// As explained in the header file, this class implements a separate
// thread that queues commands and then sends them one-by-one to the
// low-level controller whenever the low-level queries it for another
// command to execute. This design is necessitated by the fact that the
// iRobot Create's Command Module sports a single USART that must be
// multiplexed between its USB port and the robot's internal serial port.
//
// Thus, the high-level portion of the motor system is forced to buffer
// commands before despatching them to the low-level. Needless to say, if
// this buffer of pending motor commands becomes too large (because, say,
// the low-level is operating somewhat sluggishly and hasn't been issuing
// its command queries often enough), we could end up with the robot
// executing very old/outdated commands that do not at all reflect the
// current situation on the ground.
//
// Therefore, it would be a good idea to restrict the size of the pending
// commands buffer. This constant specifies the maximum number of
// commands that can be held in the pending buffer. Once this number is
// reached, new commands that get queued will result in the oldest one
// being knocked out.
static const unsigned int LOBOT_MAX_PENDING = 4 ;

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- KNOB TWIDDLING -----------------------------

/// This local class encapsulates various parameters that can be used
/// to tweak different aspects of the motor interface.
namespace {

class Params : public singleton<Params> {
   /// The robot's motors are controlled by sending commands over a
   /// serial port. This setting specifies the serial port device that
   /// should be used.
   std::string m_device ;

   /// This setting specifies the serial port communication rate.
   int m_baud_rate ;

   /// The iRobot Create/Roomba turns by using an appropriate turn
   /// radius. However, the higher layers of the Robolocust controller
   /// work in terms of a steering direction rather than a turn
   /// radius. Therefore, we have to somehow convert the high-level
   /// steering angles to corresponding low-level turn radii.
   ///
   /// This conversion is rather ad hoc. We simply perform a linear
   /// interpolation on the magnitude of the turn angle, which would
   /// be in the range [0, T] (where T is the turn_max parameter
   /// specified in the turn_arbiter section), to get a number in the
   /// range [M, m], where m and M and are, respectively, the minimum
   /// and maximum turn radii we want.
   ///
   /// The Create/Roomba supports turn radii of 0 to 2000mm. However,
   /// these min and max parameters should be smaller than the above
   /// range, i.e., min_turn_radius should be > zero and
   /// max_turn_radius should be < 2000.
   ///
   /// Both parameters are specified in millimeters.
   int m_min_turn_radius, m_max_turn_radius ;

   /// For the Roomba platform, we compute the robot's current speed
   /// by asking the Roomba how much distance it has covered since
   /// the previous update of the speed. We then divide this distance
   /// by the amount of time that elapsed since the previous update.
   ///
   /// For the most part this computation works quite well, with the
   /// computed values being quite close to the actual/requested
   /// speed. However, there is enough jitteriness in these speed
   /// readings to warrant a simple low-pass filter to smooth out the
   /// kinks.
   ///
   /// The above-mentioned low-pass filter is effected by using a
   /// weighted moving average. This setting specifies the number of
   /// previous readings that should be considered for computing this
   /// average.
   ///
   /// NOTE: To turn the speed filter off, set this config value to
   /// one.
   int m_speed_filter_size ;

   /// The Roomba platform is equipped with a pair of Sharp GP2D15 IR
   /// proximity detectors on its rear to help the robot avoid bumping
   /// into things as it backs up. However, these two sensors have to be
   /// explicitly turned on. This flag can be used to configure the rear
   /// bump sensors.
   bool m_enable_rear_bumps ;

   /// When the Roomba platform's rear bump sensors are enabled, the
   /// robot will respond to rear bump events in one of two ways:
   ///
   ///    - just stop the robot when either of the two rear bumpers is
   ///      active
   ///
   ///    - stop the robot, spin it in-place a small amount either
   ///      clockwise or counterclockwise depending on which side the
   ///      obstacle appears and then move it forward a little bit
   ///
   /// By default, the low-level controller will respond to rear bump
   /// events by simply stopping the robot. However, this flag can be
   /// used to have the robot spin and move-up as well.
   bool m_rear_bumps_spin ;

   /// Users can configure Robolocust to print useful info such as
   /// current speed and heading every time the robot's sensorimotor
   /// system gets updated. By default, the Robolocust controller
   /// prints nothing. To enable this printing, switch on this flag.
   bool m_print_status ;

   /// Private constructor because this is a singleton.
   Params() ;

   // Boilerplate code to make generic singleton design pattern work
   friend class singleton<Params> ;

public:
   /// Accessing the various parameters.
   //@{
   static const std::string& device() {return instance().m_device ;}
   static int  baud_rate()         {return instance().m_baud_rate ;}
   static int  min_turn_radius()   {return instance().m_min_turn_radius  ;}
   static int  max_turn_radius()   {return instance().m_max_turn_radius  ;}
   static int  speed_filter_size() {return instance().m_speed_filter_size;}
   static bool enable_rear_bumps() {return instance().m_enable_rear_bumps;}
   static bool rear_bumps_spin()   {return instance().m_rear_bumps_spin  ;}
   static bool print_status()      {return instance().m_print_status ;}
   //@}
} ;

// Parameters initialization
Params::Params()
   : m_device(robot_conf<std::string>("serial_port", "/dev/ttyUSB0")),
     m_baud_rate(clamp(robot_conf("baud_rate", 57600), 50, 115200)),
     m_min_turn_radius(clamp(robot_conf("min_turn_radius", 200), 100, 500)),
     m_max_turn_radius(clamp(robot_conf("max_turn_radius", 1000),
                             m_min_turn_radius + 100, 2000)),
     m_speed_filter_size(clamp(robot_conf("speed_filter_size", 10), 1, 100)),
     m_enable_rear_bumps(robot_conf("enable_rear_bumps", false)),
     m_rear_bumps_spin(robot_conf("rear_bumps_spin", false)),
     m_print_status(robot_conf("print_status", false))
{}

} // end of local anonymous namespace encapsulating above helper class

//-------------------------- INITIALIZATION -----------------------------

RoombaCM::RoombaCM(const ModelManager& mgr)
   : base(mgr, Params::device(), Params::baud_rate()),
     m_prev_time_stamp(0),
     m_speed_filter(Params::speed_filter_size()),
#ifdef LOBOT_MOTOR_DEVMODE // no Comm thread
     m_comm(0)
#else
     m_comm(new Comm(&m_serial))
#endif
{}

//-------------------------- MOTOR COMMANDS -----------------------------

// The iRobot Create/Roomba already has built-in PID, etc. and doesn't
// need (or even support?) drive commands in terms of PWM. Therefore, we
// ignore the PWM parameter.
//
// For the drive speed, since the Roomba works in mm/s and supports
// speeds of up to 500mm/s, we need to convert the input speed, which is
// in m/s, to mm/s and ensure that it is in the proper range.
void RoombaCM::drive(float speed, int)
{
   if (speed > 0)
      send_roomba(LOBOT_CMD_FORWARD, clamp(round( speed * 1000), 0, 500)) ;
   else if (speed < 0)
      send_roomba(LOBOT_CMD_REVERSE, clamp(round(-speed * 1000), 0, 500)) ;
   else { // speed == zero ==> stop the robot
      send_roomba(LOBOT_CMD_STOP) ;
      m_speed_filter.reset() ;
   }
}

// To turn the iRobot Create, we have to specify the desired turn radius
// to its drive command. However, the higher (C++) layers of the
// Robolocust controller work in terms of steering directions. Therefore,
// we need to somehow convert steering directions to appropriate turn
// radii.
//
// This method achieves the above-mentioned conversion, albeit in a
// rather ad hoc manner. We simply use a linear interpolation of (the
// absolute value of) the turn direction, which is in the range [0, T],
// where T is the maximum steering angle supported by the turn arbiter,
// to a corresponding number in the range [M, m], where m and M are the
// minimum and maximum turn radii we would like.
//
// NOTE: We go from [0, T] to [M, m], i.e., larger turn angles get
// converted to smaller turn radii. This makes sense because larger turn
// angles mean hard turns, which are effected by smaller turn radii.
// Similarly, smaller turn angles (medium or soft turns) will be effected
// by larger turn radii.
void RoombaCM::turn(float direction)
{
   const float T = TurnArbiter::turn_max() ;
   const int   m = Params::min_turn_radius() ;
   const int   M = Params::max_turn_radius() ;

   int turn_radius = round(M + abs(direction) * (m - M)/T) ;

   if (direction < 0) // negative angle ==> turn right
      send_roomba(LOBOT_CMD_RIGHT, clamp(turn_radius, 100, 2000)) ;
   else if (direction > 0) // positive angle ==> turn left
      send_roomba(LOBOT_CMD_LEFT,  clamp(turn_radius, 100, 2000)) ;
   else // direction == zero ==> drive straight ahead
      send_roomba(LOBOT_CMD_STRAIGHT) ;
}

// The Roomba platform supports in-place turns
void RoombaCM::spin(float angle)
{
   // Massage spin amount to lie within [-360, 360]
   int rotation = round(clamp_angle(angle)) ;
   if (angle < 0)
      rotation -= 360 ;

   // Then, simply pass to low-level controller
   if (rotation)
      send_roomba(LOBOT_CMD_SPIN, rotation) ;
}

void RoombaCM::off()
{
   drive(0) ;
}

//--------------------------- ROBOT STATUS ------------------------------

// Quick helper to tell whether the turn radius returned by the Roomba
// corresponds to driving straight ahead or not.
static inline bool is_straight(int radius)
{
   return (radius == LOBOT_OI_DRIVE_STRAIGHT)
       || (radius == LOBOT_OI_DRIVE_STRAIGHT2)
       || (radius == 0) ;
}

// Main thread's update cycle to retrieve robot's current state.
//
// The low-level Command Module control program keeps streaming sensor
// data to the high level. The Comm thread takes care of buffering these
// sensor packets. So, in the main thread, we only need to query the Comm
// thread for the next pending sensor packet.
#ifdef LOBOT_MOTOR_DEVMODE

bool RoombaCM::update_sensors(){return false ;}

#else

bool RoombaCM::update_sensors()
{
   bool new_sensor_packet_available = false ;

   Comm::Sensors S ;
   if (m_comm->sensors(&S))
   {
      new_sensor_packet_available = true ;
      time_stamp(S.time_stamp) ;

      int dist  = make_word(S.bytes[LOBOT_SENSORS_DISTANCE_HI],
                            S.bytes[LOBOT_SENSORS_DISTANCE_LO]) ;
      float vel = dist/static_cast<float>(S.time_stamp - m_prev_time_stamp) ;
      m_speed_filter.add(vel) ;
      speed(m_speed_filter.value()) ;
      m_prev_time_stamp = S.time_stamp ;

      const float T   = TurnArbiter::turn_max() ;
      const int   m   = Params::min_turn_radius() ;
      const int   M   = Params::max_turn_radius() ;
      int turn_radius = make_word(S.bytes[LOBOT_SENSORS_REQUESTED_RADIUS_HI],
                                  S.bytes[LOBOT_SENSORS_REQUESTED_RADIUS_LO]) ;
      if (is_straight(turn_radius))
         heading(0) ;
      else
         heading(sign(turn_radius) * (abs(turn_radius) - M) * T/(m - M)) ;

      /*
      LERROR("sensor packet [dvrt]: %5dmm %6.3fm/s %6dmm %14lldms",
             dist, vel, turn_radius, S.time_stamp) ;
      // */

      const int bumps = S.bytes[LOBOT_SENSORS_BUMPS] ;
      bump_left (bumps & LOBOT_OI_BUMP_LEFT)  ;
      bump_right(bumps & LOBOT_OI_BUMP_RIGHT) ;
      bump_rear_left (bumps & LOBOT_BUMP_REAR_LEFT)  ;
      bump_rear_right(bumps & LOBOT_BUMP_REAR_RIGHT) ;

      wheel_drop_left(
         S.bytes[LOBOT_SENSORS_WHEEL_DROPS] & LOBOT_OI_WHEEL_DROP_LEFT) ;
      wheel_drop_right(
         S.bytes[LOBOT_SENSORS_WHEEL_DROPS] & LOBOT_OI_WHEEL_DROP_RIGHT) ;
      wheel_drop_caster(
         S.bytes[LOBOT_SENSORS_WHEEL_DROPS] & LOBOT_OI_WHEEL_DROP_CASTER) ;

      wall(S.bytes[LOBOT_SENSORS_WALL]) ;
      virtual_wall(S.bytes[LOBOT_SENSORS_VIRTUAL_WALL]) ;
      wall_signal(make_uword(S.bytes[LOBOT_SENSORS_WALL_SIGNAL_HI],
                             S.bytes[LOBOT_SENSORS_WALL_SIGNAL_LO])) ;

      cliff_left (S.bytes[LOBOT_SENSORS_CLIFF_LEFT])  ;
      cliff_right(S.bytes[LOBOT_SENSORS_CLIFF_RIGHT]) ;
      cliff_front_left (S.bytes[LOBOT_SENSORS_CLIFF_FRONT_LEFT])  ;
      cliff_front_right(S.bytes[LOBOT_SENSORS_CLIFF_FRONT_RIGHT]) ;

      cliff_left_signal(
         make_uword(S.bytes[LOBOT_SENSORS_CLIFF_LEFT_SIGNAL_HI],
                    S.bytes[LOBOT_SENSORS_CLIFF_LEFT_SIGNAL_LO])) ;
      cliff_right_signal(
         make_uword(S.bytes[LOBOT_SENSORS_CLIFF_RIGHT_SIGNAL_HI],
                    S.bytes[LOBOT_SENSORS_CLIFF_RIGHT_SIGNAL_LO])) ;
      cliff_front_left_signal(
         make_uword(S.bytes[LOBOT_SENSORS_CLIFF_FRONT_LEFT_SIGNAL_HI],
                    S.bytes[LOBOT_SENSORS_CLIFF_FRONT_LEFT_SIGNAL_LO])) ;
      cliff_front_right_signal(
         make_uword(S.bytes[LOBOT_SENSORS_CLIFF_FRONT_RIGHT_SIGNAL_HI],
                    S.bytes[LOBOT_SENSORS_CLIFF_FRONT_RIGHT_SIGNAL_LO])) ;

      infrared(S.bytes[LOBOT_SENSORS_INFRARED_BYTE] & 0xFF) ;
      distance(dist) ;
      angle(make_word(S.bytes[LOBOT_SENSORS_ANGLE_HI],
                      S.bytes[LOBOT_SENSORS_ANGLE_LO])) ;
      spin_flag(S.bytes[LOBOT_SENSORS_SPIN_FLAG]) ;

      battery_charge(make_uword(S.bytes[LOBOT_SENSORS_BATTERY_CHARGE_HI],
                                S.bytes[LOBOT_SENSORS_BATTERY_CHARGE_LO])) ;
      requested_speed(make_word(S.bytes[LOBOT_SENSORS_REQUESTED_SPEED_HI],
                                S.bytes[LOBOT_SENSORS_REQUESTED_SPEED_LO])) ;
      requested_radius(turn_radius) ;
   }

   if (Params::print_status())
      LERROR("drive = [%7.3f]; turn = [%6.1f]",
             current_speed(), current_heading()) ;

   return new_sensor_packet_available ;
}

#endif

//--------------------- LOW-LEVEL MOTOR COMMANDS ------------------------

#ifdef LOBOT_MOTOR_DEVMODE

// Development mode dummy
class fake_roomba {
   short int m_speed, m_turn_radius ;
public:
   fake_roomba() ;
   void send(int cmd) ;
   void send(int cmd, short int param) ;
   int  recv(int cmd) ;
} ;

fake_roomba::fake_roomba()
   : m_speed(0), m_turn_radius(0x8000)
{}

void fake_roomba::send(int cmd)
{
   switch (cmd)
   {
      case LOBOT_CMD_STOP:
         m_speed = 0 ;
         m_turn_radius = 0x8000 ;
         break ;
      case LOBOT_CMD_STRAIGHT:
         m_turn_radius = 0x8000 ;
         break ;
   }
}

void fake_roomba::send(int cmd, short int param)
{
   switch (cmd)
   {
      case LOBOT_CMD_FORWARD:
         m_speed = param ;
         m_turn_radius = 0x8000 ;
         break ;
      case LOBOT_CMD_REVERSE:
         m_speed = -param ;
         m_turn_radius = 0x8000 ;
         break ;
      case LOBOT_CMD_LEFT:
         m_turn_radius = param ;
         break ;
      case LOBOT_CMD_RIGHT:
         m_turn_radius = -param ;
         break ;
   }
}

int fake_roomba::recv(int cmd)
{
   return -1 ;
}

static fake_roomba roomba ;

void RoombaCM::send_roomba(int cmd, int param)
{
   roomba.send(cmd, param) ;
}

#else // the real iRobot Create/Roomba interface

void RoombaCM::send_roomba(int cmd, int param)
{
   m_comm->buffer(cmd, param) ;
}

#endif // LOBOT_MOTOR_DEVMODE

//---------------- LOW-LEVEL COMMUNICATIONS INTERFACE -------------------

// Constructor
RoombaCM::Comm::Comm(lobot::Serial* S)
   : m_serial(S)
{
   start("roomba_comm_thread") ;

   if (Params::enable_rear_bumps())
      buffer(LOBOT_CMD_ENABLE_REAR_BUMPS, Params::rear_bumps_spin()) ;
}

// Command constructor
RoombaCM::Comm::Cmd::Cmd(int cmd, int param)
{
   bytes[0] = (cmd   & 0x00FF);                // the command code
   bytes[1] = (param & 0xFF00) >> 8 ;          // parameter's high byte
   bytes[2] = (param & 0x00FF) ;               // parameter's low  byte
   bytes[3] = bytes[0] ^ bytes[1] ^ bytes[2] ; // XOR parity checksum
}

// Command copy constructor
RoombaCM::Comm::Cmd::Cmd(const RoombaCM::Comm::Cmd::Cmd& C)
{
   std::copy(C.bytes, C.bytes + LOBOT_CMD_SIZE, bytes) ;
}

// Sensors constructors
RoombaCM::Comm::Sensors::Sensors()
   : time_stamp(0)
{
   std::fill(bytes, bytes + LOBOT_SENSORS_SIZE, 0) ;
}

RoombaCM::Comm::Sensors::Sensors(const char sensor_data[])
   : time_stamp(current_time())
{
   std::copy(sensor_data, sensor_data + LOBOT_SENSORS_SIZE, bytes) ;
}

// Sensors copy constructor
RoombaCM::Comm::Sensors::Sensors(const RoombaCM::Comm::Sensors& S)
   : time_stamp(S.time_stamp)
{
   std::copy(S.bytes, S.bytes + LOBOT_SENSORS_SIZE, bytes) ;
}

// Sensors assignment operator
RoombaCM::Comm::Sensors&
RoombaCM::Comm::Sensors::
operator=(const RoombaCM::Comm::Sensors& S)
{
   if (&S != this) {
      time_stamp = S.time_stamp ;
      std::copy(S.bytes, S.bytes + LOBOT_SENSORS_SIZE, bytes) ;
   }
   return *this ;
}

// Add a new command to be sent to the low-level controller to the
// internal command queue.
void RoombaCM::Comm::buffer(int cmd, int param)
{
   AutoMutex M(m_cmd_mutex) ;
   m_commands.push(Cmd(cmd, param)) ;
   if (m_commands.size() > LOBOT_MAX_PENDING)
      m_commands.pop() ;
}

// Return next pending sensor packet
bool RoombaCM::Comm::sensors(RoombaCM::Comm::Sensors* s)
{
   bool sensor_data_available = false ;

   AutoMutex M(m_sensors_mutex) ;
   if (! m_sensors.empty()) {
      *s = m_sensors.front() ;
      m_sensors.pop() ;
      sensor_data_available = true ;
   }
   return sensor_data_available ;
}

// Send next pending command to low-level Command Module control program
// for further processing by Create robot.
void RoombaCM::Comm::send_cmd()
{
   Cmd C(Pause::is_set() ? LOBOT_CMD_STOP : LOBOT_CMD_NOP) ;

   // Inner block to ensure mutex is released when no longer required
   {
      AutoMutex M(m_cmd_mutex) ;
      if (! m_commands.empty()) {
         C = m_commands.front() ;
         m_commands.pop() ;
      }
   }

   /*
   LERROR("sending [%02hhX (%c) %02hhX %02hhX %02hhX] to Command Module",
          C.bytes[0], C.bytes[0], C.bytes[1], C.bytes[2], C.bytes[3]) ;
   // */
   m_serial->write(C.bytes, LOBOT_CMD_SIZE) ;
}

// Store sensor data sent by low-level Command Module control program
void RoombaCM::Comm::recv_sensors()
{
   // Ideally, we would simply issue one call to the serial port read
   // function asking it to return the requisite number of bytes.
   // Unfortunately, that doesn't always work. Often, it reads less than
   // the full number of bytes sent by the low-level controller despite
   // the fact that all the bytes really are present in the serial
   // "stream." Then, of course, the Comm thread's main loop swallows up
   // the remaining bytes (thinking of them as bogus ACK messages from
   // the low-level instead of as bytes that are part of the sensor
   // data), which eventually leads the update function to assume that
   // no sensor data was received.
   //
   // Don't quite know why a single read doesn't work. Perhaps there's
   // some low-level handshaking going on between the Command Module's
   // USART buffer and the computer's serial buffer which causes the
   // sensor data bytes to be sent in erratic chunks? Who knows...
   //
   // Whatever the reason, the workaround is reasonably straightforward:
   // simply keep issuing serial port read operations until the required
   // number of bytes have been read. This bit of hackery seems to work
   // quite well.
   //
   // DEVNOTE: The loop to keep reading until the specified number of
   // bytes has been received is in the lobot::Serial::read_full() API.
   char sensors[LOBOT_SENSORS_SIZE] ;
   m_serial->read_full(sensors, LOBOT_SENSORS_SIZE) ;

   // Now that all the sensor data bytes are in place, we store them in a
   // RoombaCM::Comm::Sensors structure for later retrieval by the main
   // thread (see RoombaCM::update()).
   AutoMutex M(m_sensors_mutex) ;
   m_sensors.push(Sensors(sensors)) ;
   if (m_sensors.size() > LOBOT_MAX_PENDING)
      m_sensors.pop() ;
   //LERROR("%lu sensor packets pending", m_sensors.size()) ;
}

// The low-level controller sends several different acknowledgement
// messages, most of which are of little concern to the high-level
// controller (as of now, i.e., circa end-Jan 2010). So, we simply
// discard those acknowledgements.
//
// As the low level's acknowledgement messages are often accompanied by
// several data bytes, discarding the entire message involves reading
// those bytes so as to clear them from the serial port. Note that we
// could simply flush the serial port's input queue. However, that
// doesn't always work. See the comment in the recv_sensors() function.
// That is why we have to explicitly read the full number of data bytes
// for each acknowlegdement to get the whole thing out of the way.
//
// This helper class implements a map that specifies the number of data
// bytes to be read and discarded for each different acknowledgement
// message. Using this map obviates the need for a long switch-case
// construct in which every unnecessary acknowledgement is handled the
// same way, viz., by eating the requisite number of bytes.
//
// DEVNOTE: We could use an STL map here. However, since there are a
// fairly small number of entries in this map, we simply code it using an
// array and a linear search function because an STL map has extra
// overhead (implementing binary tree, etc.). If the number of
// acknowledgements becomes large, then it might make sense to
// reimplement this helper using an STL map.
class EatMap {
   // The map contains entries specifying the number of data bytes for
   // each acknowledgement message sent by the low-level controller.
   typedef std::pair<int, int> MapEntry ;
   std::vector<MapEntry> m_map ;

   // Search for the specified acknowledgement message and return the
   // corresponding number of data bytes. Returns zero if the map doesn't
   // contain the requested info (i.e., bad/unknown ack ID).
   int lookup(int ack_id) const ;

public:
   // Inititalize the map.
   EatMap() ;

   // Check if the map contains the number of bytes corresponding to the
   // given acknowledgement message.
   bool contains(int ack_id) const {return lookup(ack_id) > 0 ;}

   // Retrieve the number of data bytes for the given acknowledgement.
   int operator[](int ack_id) const {return lookup(ack_id) ;}
} ;

// Init the map showing the number of data bytes for each acknowledgement
// message sent by the low-level controller.
//
// DEVNOTE: Every time we add a new acknowledgement, remember to update
// this function!
EatMap::EatMap()
{
   m_map.push_back(MapEntry(LOBOT_ACK_SENSORS,     LOBOT_SENSORS_SIZE)) ;
   m_map.push_back(MapEntry(LOBOT_ACK_WHEEL_DROPS, LOBOT_WHEEL_DROPS_SIZE)) ;
   m_map.push_back(MapEntry(LOBOT_ACK_CLIFFS,      LOBOT_CLIFFS_SIZE)) ;
   m_map.push_back(MapEntry(LOBOT_ACK_BUMPS,       LOBOT_BUMPS_SIZE)) ;
   m_map.push_back(MapEntry(LOBOT_ACK_REMOTE,      LOBOT_REMOTE_SIZE)) ;
}

// Look for the number of data bytes corresponding to the given
// acknowledgement. If the map doesn't have this info, return zero.
int EatMap::lookup(int ack_id) const
{
   const int n = m_map.size() ;
   for  (int i = 0; i < n; ++i)
      if (m_map[i].first == ack_id)
         return m_map[i].second ;
   return 0 ;
}

// A quick helper function to print a message letting the user know that
// the low-level controller sent some screwy acknowledgement message.
static inline void bogus_ack(char c)
{
   LERROR("received bogus ACK: 0x%02hhX (%hhd '%c')",
          c, c, isprint(c) ? c : '?') ;
}

// The Comm thread's raison d'etre: wait for the low-level controller to
// send a READY message and then, in return, send it the next pending
// high-level motor command. Additionally, the Comm thread also responds
// to acknowledgement messages from the low-level controller that send
// sensor data and notifications of low-level actions (such as stopping
// the motors in response to wheel drops).
//
// DEVNOTE: Since all Robolocust threads need to periodically check the
// system-wide shutdown signal, we cannot blithely issue a blocking read
// on the serial port to wait for the low-level control program to send
// the READY and other acknowledgement messages. If we do that, we could
// potentially hold up the lobot controller's shutdown sequence. Here's
// how:
//
// Let's say the low-level control program dies (e.g., the user either
// deliberately or inadvertently presses the Command Module's Reset
// button or powers off the Create robot). Now, this sender thread here
// will remain blocked forever waiting for messages from the low level
// that will never arrive.
//
// In the meantime, let's say the user kills the lobot application. All
// the other threads will shutdown. But this one won't because it's still
// stuck in a blocking read operation. The main thread will also keep
// waiting for this one to exit, which, of course, will never happen. And
// so the application remains stuck.
//
// NOTE: Of course, the user can always kill -9 and force the process to
// die. But that's a rather nasty non-solution. :)
//
// The long and short of it is that this function needs to first check if
// there is actually any data available on the serial port before
// performing a blocking read. If there is no data, it can sleep a short
// while and repeat its loop, thereby continuing to check the shutdown
// signal.
//
// DEVNOTE 2: Actually, the shutdown fiasco described above can still
// occur if something goes wrong between the data availability check and
// the attempt to read it that causes the data to disappear and the read
// to block forever. However, the chances of that happening are
// practically (not entirely mind you) zero. But the chances of a
// malfunctioning low-level as described above are mucho high; high
// enough to warrant some sort of safeguard against an unruly thread
// refusing to heed the shutdown signal.
void RoombaCM::Comm::run()
{
   // Create the map specifying the number of data bytes for each
   // acknowledgement so that we know how many bytes to discard for the
   // acks in which we are not interested.
   const EatMap eat_map ;

   // Comm thread main loop: retrieve acks from low-level controller and
   // respond appropriately.
   while (! Shutdown::signaled())
   {
      try
      {
         if (m_serial->ready()) // don't block! else can miss shdn signal
         {
            char c = 0 ;
            if (m_serial->read(&c, 1) >= 1)
               switch (c)
               {
                  case LOBOT_ACK_READY:
                     send_cmd() ;
                     break ;
                  case LOBOT_ACK_SENSORS:
                     recv_sensors() ;
                     break ;
                  default: {
                     const int n = eat_map[c] ;
                     if (n > 0)
                        m_serial->eat(n) ;
                     else
                        bogus_ack(c) ;}
                     break ;
               }
         }
      }
      catch (uhoh& e)
      {
         LERROR("%s encountered an error: %s", name().c_str(), e.what()) ;
         // non-fatal error (?)
         // just keep going (?)
      }
      usleep(50000) ;
   }

   // The stop command sent as part of the high-level controller's
   // shutdown sequence will probably not get sent because the Comm
   // thread might well have exited by the time the main program gets
   // around to sending the final stop command. So, we take care of that
   // here in the Comm thread right before it exits...
   //
   // NOTE: We can't just send the stop command to the low level whenever
   // we feel like! We must first wait for the low level to send an
   // ACK_READY. This means we must use a loop pretty much identical to
   // the one above. However, since shutdown has been signaled, this loop
   // should not go on for too long...
   //
   // NOTE 2: Actually, we might want to send more than just the STOP
   // command before winding down the high-level controller. To be able
   // to do that conveniently, we simply buffer the shutdown/quit/cleanup
   // commands and send them out one-by-one.
   //
   // However, when shutting down, we don't want other threads continuing
   // to buffer commands. Therefore, we lock the commands buffer before
   // beginning the shutdown sequence. Furthermore, because the buffer()
   // and send_cmd() functions also internally lock the commands buffer,
   // we cannot reuse them here. Instead, we must reimplement the
   // necessary parts of their functionality right here.

   // Lock commands buffer and clear it so that no other threads can send
   // any more commands to low-level.
   AutoMutex M(m_cmd_mutex) ;
   while (! m_commands.empty())
      m_commands.pop() ;

   // Buffer commands to be sent to low-level as part of the shutdown
   // sequence...
   m_commands.push(Cmd(LOBOT_CMD_STOP)) ;
   if (Params::enable_rear_bumps())
      m_commands.push(Cmd(LOBOT_CMD_DISABLE_REAR_BUMPS)) ;

   // Wait for ACK_READY from low-level and send above buffered commands
   const int MAX_TRIES = 10 ;
   for  (int i = 0; i < MAX_TRIES; ++i)
   {
      try
      {
         if (m_serial->ready())
         {
            char c = 0 ;
            if (m_serial->read(&c, 1) >= 1)
               switch (c)
               {
                  case LOBOT_ACK_READY: {
                     // Send next shutdown related command to low-level
                     Cmd C(m_commands.front()) ;
                     m_commands.pop() ;
                     m_serial->write(C.bytes, LOBOT_CMD_SIZE) ;
                     if (m_commands.empty()) // all shdn related commands sent
                        return ;} // quit low-level comm interface thread
                     break ;
                  default: { // not interested in any other ACKs during shdn
                     const int n = eat_map[c] ;
                     if (n > 0)
                        m_serial->eat(n) ;
                     else
                        bogus_ack(c) ;}
                     break ;
               }
         }
      }
      catch (uhoh& e)
      {
         LERROR("%s encountered an error: %s", name().c_str(), e.what()) ;
         // non-fatal error (?)
         // just keep going (?)
      }

      usleep(50000) ;
   }
   LERROR("%s: on shdn, unable to send '%c' cmd; low-level dead?",
          name().c_str(), m_commands.front().bytes[0]) ;
}

// Low-level communications interface thread clean-up
RoombaCM::Comm::~Comm(){}

//----------------------------- CLEAN-UP --------------------------------

RoombaCM::~RoombaCM(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
