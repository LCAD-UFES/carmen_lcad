{
   Robots/LoBot/spin/LoMotorDriver.spin

   This file contains the code for lobot's Propeller board, which is
   hooked up to the mini-ITX box via USB. The computer controls the
   robot's motors by sending appropriate commands to the Propeller,
   which, in turn, commands the Sabertooth motor driver to do its thing.

   The Propeller simply acts as a relay between the computer and the
   Sabertooth. That is, the high-level C++ layers of the system must
   perform the necessary computations and formatting of the Sabertooth
   commands. The Propeller will only pass those bytes on to the
   Sabertooth as-is.
}

{
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
}

' Primary maintainer for this file: Manu Viswanathan <mviswana@usc.edu>
' $HeadURL$
' $Id$

'------------------------------ CONSTANTS -------------------------------

CON
   ' Clock parameters
   _clkmode = xtal1 + pll16x
   _xinfreq = 5_000_000

   ' Serial port parameters
   RX_PIN    = 31
   TX_PIN    = 30
   BAUD_RATE = 115200

   ' In case the low-level motor interface doesn't hear anything from the
   ' higher layers of the system for the following number of
   ' milliseconds (e.g., the C++ controller has crashed), it will
   ' automatically issue the OFF command. Thus, the higher level parts of
   ' the robot's controller must keep talking to the Propeller...
   COMPUTER_TIMEOUT = 1000

   ' Sabertooth parameters
   ST_TXPIN   = 19
   ST_BAUD    = 2400
   ST_ADDRESS = 128

   ' Sabertooth commands
   ST_MOTOR1_FWD  = 0
   ST_MOTOR2_FWD  = 4
   ST_MOTOR1_REV  = 1
   ST_MOTOR2_REV  = 5
   ST_MOTOR1_STOP = 6
   ST_MOTOR2_STOP = 7

   ' Steering servo parameters
   STEERING_PIN = 16

   ' RPM parameters
   RPM_PIN     = 18
   RPM_TIMEOUT = 1000

   ' The different high-level commands this object recognizes
   FORWARD_CMD = 100
   REVERSE_CMD = 101
   STOP_CMD    = 102

   LEFT_CMD     = 110
   RIGHT_CMD    = 111
   STRAIGHT_CMD = 112

   OFF_CMD = 120

   ' Commands to return status and other info
   GET_MOTOR_DIR = 130
   GET_MOTOR_PWM = 131
   GET_SERVO_DIR = 132
   GET_SERVO_PWM = 133
   GET_RPM       = 134

'------------------------------- OBJECTS --------------------------------

OBJ
   computer   : "FullDuplexSerial"
   sabertooth : "Simple_Serial"
   steering   : "Servo32v7"
   rpm        : "LoRPM"        ' object to measure robot wheel's RPM

'-------------------------- GLOBAL VARIABLES ----------------------------

VAR
   byte motor_dir
   byte motor_pwm
   byte servo_pwm
   byte servo_dir

'-------------------------------- MAIN ----------------------------------

PUB main | cmd
   ' Start the serial port object that interfaces to the robot's computer
   computer.start(RX_PIN, TX_PIN, 0, BAUD_RATE)

   ' Start the serial port object that interfaces to the Sabertooth
   sabertooth.init(0, ST_TXPIN, ST_BAUD)
   sabertooth.tx(170) ' must send this to Sabertooth before any other cmds

   ' Start the servo object that interfaces with the steering servo
   steering.start
   steering.ramp

   ' Start the RPM measuring object
   rpm.start(RPM_PIN, RPM_TIMEOUT)

   ' Main loop: listen to the serial port for incoming commands and
   ' execute them.
   repeat
      cmd := computer.rxtime(COMPUTER_TIMEOUT)
      if (cmd < 0) ' no command from computer for quite some time
         cmd := OFF_CMD

      case cmd
         FORWARD_CMD:
            forward(computer.rx)

         REVERSE_CMD:
            reverse(computer.rx)

         STOP_CMD:
            stop

         LEFT_CMD:
            left(computer.rx)

         RIGHT_CMD:
            right(computer.rx)

         STRAIGHT_CMD:
            straight

         OFF_CMD:
            straight
            stop
            'sabertooth.finalize
            'computer.stop

         GET_MOTOR_DIR:
            computer.tx(motor_dir)

         GET_MOTOR_PWM:
            computer.tx(motor_pwm)

         GET_SERVO_DIR:
            computer.tx(servo_dir)

         GET_SERVO_PWM:
            computer.tx(servo_pwm)

         GET_RPM:
            send_rpm

'-------------------------- DRIVING COMMANDS ----------------------------

PRI forward(speed)
   if (speed < 0)
      speed := 0
   elseif (speed > 127)
      speed := 127

   sabertooth.tx(ST_ADDRESS)
   sabertooth.tx(ST_MOTOR1_FWD)
   sabertooth.tx(speed)
   sabertooth.tx((ST_ADDRESS + ST_MOTOR1_FWD + speed) & %01111111)

   sabertooth.tx(ST_ADDRESS)
   sabertooth.tx(ST_MOTOR2_FWD)
   sabertooth.tx(speed)
   sabertooth.tx((ST_ADDRESS + ST_MOTOR2_FWD + speed) & %01111111)

   motor_dir := FORWARD_CMD
   motor_pwm := speed

PRI reverse(speed)
   if (speed < 0)
      speed := 0
   elseif (speed > 127)
      speed := 127

   sabertooth.tx(ST_ADDRESS)
   sabertooth.tx(ST_MOTOR1_REV)
   sabertooth.tx(speed)
   sabertooth.tx((ST_ADDRESS + ST_MOTOR1_REV + speed) & %01111111)

   sabertooth.tx(ST_ADDRESS)
   sabertooth.tx(ST_MOTOR2_REV)
   sabertooth.tx(speed)
   sabertooth.tx((ST_ADDRESS + ST_MOTOR2_REV + speed) & %01111111)

   motor_dir := REVERSE_CMD
   motor_pwm := speed

PRI stop
   forward(0)
   motor_dir := STOP_CMD
   motor_pwm := 0

'-------------------------- STEERING COMMANDS ---------------------------

PRI left(amount)
   if (amount < 0)
      amount := 0
   elseif (amount > 100)
      amount := 100

   steering.set(STEERING_PIN, 1000 + 5 * (100 - amount))

   servo_dir := LEFT_CMD
   servo_pwm := amount

PRI right(amount)
   if (amount < 0)
      amount := 0
   elseif (amount > 100)
      amount := 100

   steering.set(STEERING_PIN, 1500 + 5 * amount)

   servo_dir := RIGHT_CMD
   servo_pwm := amount

PRI straight
   steering.set(STEERING_PIN, 1500)

   servo_dir := STRAIGHT_CMD
   servo_pwm := 0

'---------------------------- RPM RETRIEVAL -----------------------------

PRI send_rpm | current_rpm
   current_rpm := rpm.getMiliRPM
   computer.tx((current_rpm >> 24) & $FF)
   computer.tx((current_rpm >> 16) & $FF)
   computer.tx((current_rpm >>  8) & $FF)
   computer.tx(current_rpm & $FF)
