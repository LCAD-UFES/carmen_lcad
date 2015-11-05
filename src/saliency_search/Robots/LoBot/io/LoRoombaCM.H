/**
   \file  Robots/LoBot/io/LoRoombaCM.H
   \brief Interface for driving the iRobot's Create/Roomba via its
   Command Module (hence the CM in the class name).

   This file defines a class that implements the sensorimotor interface
   defined in LoRobot.H for the Create/Roomba.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoRoombaCM.H $
// $Id: LoRoombaCM.H 13770 2010-08-08 06:49:50Z mviswana $
//

#ifndef LOBOT_ROOMBA_CM_DOT_H
#define LOBOT_ROOMBA_CM_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoRobot.H"
#include "Robots/LoBot/io/LoSerial.H"

#include "Robots/LoBot/thread/LoMutex.H"
#include "Robots/LoBot/thread/LoThread.H"

#include "Robots/LoBot/misc/factory.hh"
#include "Robots/LoBot/misc/wma.hh"

#include "Robots/LoBot/irccm/LoCMInterface.h" // iface to low-level controller

// INVT model manager stuff
#include "Component/ModelManager.H"

// Standard C++ headers
#include <string>
#include <queue>
#include <memory>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::RoombaCM
   \brief High-level API for driving and steering the Roomba and
   retrieving its sensor data.

   This class sends the drive and turn commands used by the Robolocust
   speed and turn arbiters to the Roomba over a serial port. It expects
   that the other end of the serial port is connected to an iRobot Create
   Command Module (hence the "CM" moniker in this class's name) running
   the low-level Robolocust control program, which listens for high-level
   motor commands to come in and then executes them in terms of the motor
   primitives actually supported by the Roomba.

   In addition to sending motor control commands, this class also
   retrieves the Roomba's sensor data.
*/
class RoombaCM : public Robot {
   // Handy type to have around in a derived class
   typedef Robot base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<RoombaCM, base, ModelManager> ;
   typedef register_factory<RoombaCM, base, ModelManager> my_factory ;
   static  my_factory register_me ;

   /// We update the robot's current speed by querying the Create/Roomba
   /// for the distance traveled since the last update and then dividing
   /// that distance by the time since the last update. Obviously, for
   /// this to work properly, we will need to store the time stamp of
   /// the most recent sensor packet update. This data member does just
   /// that.
   long long m_prev_time_stamp ;

   /// For the most part, gauging the robot's current speed by dividing
   /// the distance traveled since last update by time elapsed since
   /// last update works surprisingly well. However, there is still a
   /// fair amount of noisiness to the computed speed values. We use a
   /// low-pass filter to stabilize these computations.
   wma<float> m_speed_filter ;

   /// Private constructor because the interface object for a robot's
   /// motor subsystem is created using a factory.
   ///
   /// DEVNOTE: We need an INVT ModelManager because INVT's serial
   /// communications class requires it.
   RoombaCM(const ModelManager&) ;

   /// Clients must issue drive commands using both a speed expressed in
   /// m/s and a PWM value. The Roomba doesn't require or use the PWM
   /// value. However, as other robot classes do, it is part of the
   /// interface.
   void drive(float speed, int pwm = 0) ;

   /// Command the robot to turn towards the specified direction
   /// (expressed in degrees). Zero degrees is for driving straight
   /// ahead; positive (ccw) angles are for heading left and negative
   /// (cw) angles for heading right.
   void turn(float direction) ;

   /// Command the robot to spin in-place by the specified angle
   /// (expressed in degrees). Positive angles result in ccw in-place
   /// turns while negative angles are for cw in-place turns.
   void spin(float angle) ;

   /// Shut off the robot's drive motors.
   void off() ;

   /// Get the current speed, heading and other sensor values supported
   /// by the robot platform.
   bool update_sensors() ;

   /// Send a high-level command plus its parameter to the low-level
   /// control program running on the Command Module. (This function
   /// actually buffers the command with the Comm thread.)
   void send_roomba(int cmd, int param = 0) ;

   /// The interface to the low-level control program running on the
   /// iRobot Create/Roomba's Command Module works as follows: it first
   /// sends a READY acknowledgement over its USB port and then waits for
   /// a response. The higher layers of the Robolocust controller (viz.,
   /// this class's instance) must then send back the command it wants
   /// the robot to execute.
   ///
   /// Each such command consists of four bytes. The first byte is the
   /// motor command or sensor query ID. The next two bytes are the
   /// parameter for the command (e.g., drive forward at 100 mm/s). The
   /// last byte is a checksum: basically, just a parity byte computed by
   /// XOR-ing the other three bytes.
   ///
   /// If the low-level controller doesn't get back its 4-byte response
   /// to its ACK_READY message within some predefined timeout limit
   /// (compiled into the low-level program; usually, in the order of one
   /// to two seconds), it will assume that the higher layers are dead
   /// and stop the robot (until it hears from the high level again).
   ///
   /// We need this somewhat convoluted architecture because the Command
   /// Module's ATmega168 microcontroller only has one USART that can
   /// either talk to the USB port or to the robot's internal serial
   /// port. Therefore, we have to keep switching the USART between the
   /// two destinations and must ensure that the high-level portions of
   /// lobot's controller don't send commands when the USART is busy
   /// talking to the robot.
   ///
   /// Consequently, we have the Command Module's control program
   /// initiate communication with the higher layers by sending a READY
   /// message and waiting for a response. That way, we can be sure the
   /// low level is listening to the high level and that commands don't
   /// get lost somehow.
   ///
   /// Naturally, to make this work, the high level must run a dedicated
   /// thread for listening to incoming READY messages and responding to
   /// them appropriately. The following inner class implements this
   /// thread. It queues commands to be sent to the robot and sends the
   /// next pending command each time it gets an ACK_READY from the
   /// low-level control program.
   class Comm : private Thread {
      /// Each command sent to the low-level controller consists of four
      /// bytes. The first byte is the command code, the next two bytes
      /// are for a parameter, and the last byte is a parity byte
      /// checksum. This structure holds these four bytes together.
      ///
      /// DEVNOTE: Instead of hard-coding the size of the char array used
      /// to hold the byte sequence making up a high-level command, we
      /// prefer the use of the LOBOT_CMD_SIZE enum defined in
      /// irccm/LoCMInterface.h (just in case this ever changes in the
      /// future, e.g., command + params + time stamp + checksum).
   public:
      struct Cmd {
         char bytes[LOBOT_CMD_SIZE] ;

         Cmd(int cmd = 0, int param = 0) ;
         Cmd(const Cmd&) ;
      } ;

      /// The Comm thread is responsible for queueing commands to be sent
      /// to the lower levels of the robot's controller and waiting for
      /// the low-level control program to request the next such command
      /// in the queue.
      ///
      /// This data structure implements the above-mentioned command
      /// queue.
      //@{
   private:
      typedef std::queue<Cmd> CmdQueue ;
      CmdQueue m_commands ;
      //@}

      /// The Comm thread's command queue will be accessed by the Comm
      /// thread itself as well as by the arbiters and (possibly) the
      /// main thread. Therefore, we need to protect simultaneous
      /// accesses to it.
      Mutex m_cmd_mutex ;

      /// The sensor data is received from the low-level control program
      /// in a series of bytes. This structure holds these bytes together
      /// in the incoming sensor data buffer (see typedef below).
      ///
      /// DEVNOTE: The number of sensor data bytes sent, viz.,
      /// LOBOT_SENSORS_SIZE, is defined in irccm/LoCMInterface.h.
   public:
      struct Sensors {
         char bytes[LOBOT_SENSORS_SIZE] ;
         long long time_stamp ;

         Sensors() ;
         Sensors(const char sensor_data[]) ;
         Sensors(const Sensors&) ;
         Sensors& operator=(const Sensors&) ;
      } ;

      /// Just like the outgoing commands, the incoming sensor data must
      /// also be buffered. This data structure takes care of buffering
      /// the incoming sensor data.
      //@{
   private:
      typedef std::queue<Sensors> SensorQueue ;
      SensorQueue m_sensors ;
      //@}

      /// The sensor data buffer can be accessed by the main thread
      /// (during its update cycle) and by the Comm thread (when it
      /// receives sensor data from the low level). Thus, we need to
      /// properly synchronize accesses to the buffer.
      Mutex m_sensors_mutex ;

      /// In order to actually send and receive data to/from the iRobot
      /// Create Command Module, we will need a serial port for the
      /// requisite I/O operations.
      lobot::Serial* m_serial ;

   public:
      /// Initialization
      Comm(lobot::Serial*) ;

      /// The motor commands issued by the Robolocust arbiters will first
      /// be buffered by the Comm thread with this API.
      void buffer(int cmd, int param = 0) ;

      /// During its update cycle, the main thread can retrieve the
      /// Create robot's sensor data with this API. This function will
      /// return false if there is no sensor data available; true
      /// otherwise. Thus, the caller should check the return value
      /// before attempting to use the sensor data returned via the
      /// function's pass-by-reference parameter.
      bool sensors(Sensors*) ;

   private:
      /// This method sends the next pending command from the Comm
      /// thread's command buffer to the low-level Command Module control
      /// program for further processing by the Create robot.
      void send_cmd() ;

      /// This method receives the Create/Roomba sensor data sent by the
      /// low-level control program running on the Command Module.
      void recv_sensors() ;

      /// This method implements the Comm thread's main loop, wherein it
      /// listens to the Command Module's serial connection for ACK
      /// messages from the low-level control program and responds as
      /// required.
      void run() ;

   public:
      /// Clean-up
      ~Comm() ;
   } ;// end of inner Comm class

   /// The interface between the higher layers and the low-level portions
   /// of the Robolocust controller (when it is running on an iRobot
   /// Create/Roomba) need to go through an instance of the Comm thread
   /// class defined above.
   std::auto_ptr<Comm> m_comm ;

   /// Clean-up.
  ~RoombaCM() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
