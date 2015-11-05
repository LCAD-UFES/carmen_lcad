{
   Robots/LoBot/spin/LoSabertooth.spin

   lobot uses a Sabertooth motor driver (from Dimension Engineering) to
   supply current and voltage to the Traxxas R/C car's motors. The
   Sabertooth is setup to accept commands in packetized serial mode from
   a Propeller board.

   This file contains the code for a Propeller object that encapsulates
   all of the details of the communication between Propeller and
   Sabertooth. This object was originally developed by the Beobot 2.0
   team and has been copied and modified to work with lobot.
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

' Primary maintainer for this file: Dicky Nauli Sihite <sihite@usc.edu>
' $HeadURL$
' $Id$

'------------------------------ CONSTANTS -------------------------------

CON
   _clkmode = xtal1 + pll16x
   _xinfreq = 5_000_000

   RAMP_INC = 5

   CH_ROTATION   = 0 'Steering 1117~1560~2090 (left, center, right)
   CH_SPEED      = 1 'Gas      1232~1417~2019 (70/30, 50/50, pressed)
   CH_ESTOP      = 2 'Switch   1100,1530~1819 (up pressed, down pressed)
   'CH_SPEED_CAP = 3 '?        n/a
   'CH_VR        = 5 '?        n/a
   'CH_MODE      = 6 '?        n/a

   'Pin assignments
   TOTAL_CH      = 3
   CH_0          = 23
   CH_1          = 22
   CH_2          = 21
   SteeringServo = 16
   Servo_0       = 0
   Servo_1       = 1
   Servo_2       = 2

'----------------------------- DATA BLOCK -------------------------------

DAT
   remote_pins        LONG CH_0, CH_1, CH_2
   remote_pulseWidths LONG 1, 1, 1
   servo_pins         LONG Servo_0, Servo_1, Servo_2
   servo_value        LONG 64, 64, 64

'-------------------------- GLOBAL VARIABLES ----------------------------

VAR
   long stack[128]
   long motor_stack[20]
   long motorspeed

   long rcspeed
   long rcdirection
   long timestamp
   byte remoteMode
   byte motorEnable
   long emergencyMode
   long rot_vel_req        'rot request from computer
   long tran_vel_req       'speed request from computer
   long rc_rot_vel         'rc turn value times rot cap
   long rc_trans_vel       'rc speed value times speed cap
   long current_speed      'current speed that set to motor
   long current_rot        'current rotation that set to motor
   long ramp_cnt

'------------------------------- OBJECTS --------------------------------

OBJ
   SabertoothSerial : "Simple_Serial"
   Serial           : "Simple_Serial"
   remote           : "ServoInput"
   Servo            : "Servo32v7.spin"

'-------------------------- PUBLIC INTERFACE ----------------------------

'Start up the sabertooth runtime controller (in a new cog)
PUB start(EStopPin, TxPin, Address, Timeout) : success | cog
   success := (cog := cognew(SaberToothRuntime(EStopPin, TxPin, Address, Timeout), @stack) + 1)

PUB GetRemoteMode : mode
   timestamp := cnt
   return remoteMode

PUB GetChannel(chan) : chanVal
   timestamp := cnt
   return remote_pulseWidths[chan]

PUB GetEnabled : en
   timestamp := cnt
   return motorEnable

PUB GetRCSpeed : rcs
   timestamp := cnt
   return rcspeed

PUB GetEmergencyMode : em
   timestamp := cnt
   return emergencyMode

PUB GetMotorSpeed :spd
   timestamp := cnt
   return motorspeed

PUB GetRotationalSpeed
   timestamp := cnt
   return current_rot

PUB GetTranslationalSpeed
   timestamp := cnt
   return current_speed

PUB GetRcRotationalSpeed
   timestamp := cnt
   return rc_rot_vel

PUB GetRcTranslationalSpeed
   timestamp := cnt
   return rc_trans_vel

PUB SetRotationalSpeed(speed)
   if (speed > 100)
      speed := 100
   elseif (speed < -100)
      speed := -100
   rot_vel_req := Speed
   timestamp := cnt 'reset watchdog timestamp

PUB SetTranslationalSpeed(speed)
   if (speed > 100)
      speed := 100
   elseif (speed < -100)
      Speed := -100
   tran_vel_req := Speed
   timestamp := cnt 'reset watchdog timestamp

'-------------------- SABERTOOTH RUN-TIME INTERNALS ---------------------

' The sabertooth runtime controller - takes care of servicing speed
' change requests, and minding the timeout.
PRI SaberToothRuntime(EStopPin, TxPin, Address, Timeout) | speed, direction, rc_speed_cap, trans_vel, rot_vel, rot_vel_cap, targetSpeed, estop, emergencyTimer,lcdcount,channel, AutoTimer' emergencyMode

  'Turn off the e-stop
  dira[EStopPin] := 1
  outa[EStopPin] := 1

  motorspeed  := 64
  targetSpeed := 64

  'Initialize the serial port
  SabertoothSerial.init(0, TxPin, 2400)
  Serial.init(31, 30, 115200)
  SabertoothSerial.tx (170)

  remote.start(@remote_pins,TOTAL_CH,@remote_pulseWidths)
  Servo.Start
  Servo.Ramp 'Enable ramping speed

  estop := True
  emergencyMode := True

  repeat

    'All 3 channel validation
    'No channel shall be under 900 or over 2300
    repeat channel from 0 to 2
      if(remote_pulseWidths[channel] < 900 or remote_pulseWidths[channel] > 2300)
        estop := True
        emergencyMode := True

    'Determine the remote mode
    'if(remote_pulseWidths[CH_MODE] > 2500 or remote_pulseWidths[CH_MODE] <30)
    '        remoteMode := -1 'it's out of range
    'if(remote_pulseWidths[CH_MODE] > 1900 or remote_pulseWidths[CH_MODE] <700)
    '        remoteMode := 1 'Manual Mode
    'elseif(remote_pulseWidths[CH_MODE] > 1300 and remote_pulseWidths[CH_MODE] < 1700)
    '        remoteMode := 2 'Semi-Auto Mode
    'elseif(remote_pulseWidths[CH_MODE] > 700 and remote_pulseWidths[CH_MODE] < 1300)
    '        remoteMode := 3 'Auto Mode
    ' Manual mode if the the throttle is pressed OR leave in 50/50
    ' Auto mode if the throttle is left in 70/30 position
    if(remote_pulseWidths[CH_SPEED] > 1300 and remote_pulseWidths[CH_SPEED] <2300)
      remoteMode := 1 'Manual Mode
    elseif (remote_pulseWidths[CH_SPEED] > 1000 and remote_pulseWidths[CH_SPEED] < 1300)
      remoteMode := 2 'Auto Mode

    {
    'Calculate the speed limit for the robot
    rc_speed_cap := remote_pulseWidths[CH_SPEED_CAP]
    rc_speed_cap := 100-((rc_speed_cap - 1041) * 100)/(1750 - 1041)
    if(rc_speed_cap > 100)
            rc_speed_cap := 100
    if(rc_speed_cap < 0)
            rc_speed_cap := 0

    'Calculate the speed limit for the robot
    rot_vel_cap := ((remote_pulseWidths[CH_VR] - 900) * 100)/(2073- 900)
    if(rot_vel_cap> 100)
            rot_vel_cap:= 100
    if(rot_vel_cap< 0)
            rot_vel_cap:= 0
    }

    'Recover from emergency state
    'The user can get out of emergency mode by holding down the emergency stop
    'button for two seconds with the speed cap at 0
    if(emergencyMode == True)
      waitcnt(clkfreq/2+cnt)
      if(estop == False or remote_pulseWidths[CH_SPEED] > 1500 or remote_pulseWidths[CH_ROTATION] > 1700 or remote_pulseWidths[CH_ROTATION] < 1300)
        emergencyTimer := cnt
      elseif(cnt > emergencyTimer+clkfreq/4)
        repeat while remote_pulseWidths[CH_ESTOP] > 1200
        emergencyMode := False
        waitcnt(clkfreq/2+cnt)

    'Remote Kill Switch!!! Very important
    if(remote_pulseWidths[CH_ESTOP] > 1200)
      estop := True
    else
      estop := False

    if(estop == True and emergencyMode == False)
      emergencyMode := True
    ''''======================Finish Check=================================''''

    ''''Grab the translational velocity stick
    trans_vel := remote_pulseWidths[CH_SPEED]
    'trans_vel := (200 - ((trans_vel - 1041) * 200)/(1874 - 1041))-100
    trans_vel := (||(trans_vel - 1417)* 100)/(2019 - 1417)

    'Kill the middle range of the stick
    if(trans_vel < 8 and trans_vel > -8)
            trans_vel := 0

    ''''Grab the turning of stick
    rot_vel := remote_pulseWidths[CH_ROTATION]
    'rot_vel := (200 - ((rot_vel - 951) * 200)/(1780 - 951))-100
    ''rot_vel := 100 - ((rot_vel - 1117) * 200 / (2090 - 1117))
    if (rot_vel > 1560)
        rot_vel := ((rot_vel - 1560)* 100)/(2090 - 1560)
    else
        rot_vel := ((rot_vel - 1560)* 100)/(1560 - 1117)

    'Kill the middle range of the stick
    if(rot_vel < 8 and rot_vel > -8)
      rot_vel := 0

    '''' Set RC rot and trans value
    rc_rot_vel := rot_vel
    rc_trans_vel := trans_vel

    '=============== 3 Mode Option Manual/Semi-Auto/Auto==========================
    'If we are in fully manual mode:
    if(remoteMode == 1)
      rot_vel := remote_pulseWidths[CH_ROTATION]
      if (rot_vel > 1560)
        rot_vel := ((rot_vel - 1560)* 100)/(2090 - 1560)
      else
        rot_vel := ((rot_vel - 1560)* 100)/(1560 - 1117)

      'Kill the middle range of the stick
      if(rot_vel < 8 and rot_vel > -8)
              rot_vel := 0

    'If we are in any kind of auto mode:
    elseif( remoteMode > 1)
      'Check to see if our timeout has expired, or we are in a motor disabled state
      if((cnt > timestamp + Timeout))
        'If the timeout has expired, then kill the motors
        targetSpeed := 64
        tran_vel_req := 0
        rot_vel_req  := 0
        trans_vel    := 0
        rot_vel      := 0
        timestamp    := cnt

      'If we're in semi-auto mode, then only pass the rotational speed through
      'if( remoteMode == 2 )
      '  rot_vel := rot_vel_req

      'If we're in fully auto mode, then pass the requested speeds through
      if(remoteMode == 2)
        rot_vel   := rot_vel_req
        trans_vel := tran_vel_req

    '================================================================================

    'Check Speed Limit
    if(trans_vel < -100)
      trans_vel := -100
    elseif(trans_vel > 100)
      trans_vel := 100

    if(rot_vel < -100)
      rot_vel := -100
    elseif(rot_vel > 100)
      rot_vel := 100

    'Limit rotation speed by rot_vel_cap

    'targetSpeed :=  ((trans_vel * rc_speed_cap)/100 + (rot_vel*rot_vel_cap)/100)
    targetSpeed := trans_vel

    'Convert to motor driver packet serial value 0~64~127 for -100~0~100
    targetSpeed := (targetSpeed + 100) * 128 / 200

    'Report motor speed and rc speed
    rc_rot_vel    := rc_rot_vel
    rc_trans_vel  := rc_trans_vel
    'rc_rot_vel := 99
    'rc_trans_vel :=99
    current_speed := trans_vel
    current_rot   := rot_vel

    'rampMotor(targetSpeed,emergencyMode)
    motorspeed := targetSpeed
    setMotor(motorspeed,emergencyMode)
    SetServo(rot_vel,SteeringServo,emergencyMode)


PRI rampMotor(target_m, emergency)|temp_m, current_m, d

  if (target_m < motorspeed)
    motorspeed := motorspeed - RAMP_INC
  if (target_m > motorspeed)
    motorspeed := motorspeed + RAMP_INC

  if (||(motorspeed - target_m) < RAMP_INC)
    motorspeed := target_m


PRI setMotor(m_,emergency)|m_cmd
  m_cmd := m_

  if(m_cmd > 127)
    m_cmd := 127
  elseif(m_cmd < 0)
    m_cmd := 0

  if(emergency == True)
    m_cmd := 64

  motorspeed := m_cmd

  SabertoothSerial.tx(170)

  'Drive _motor 1
  SabertoothSerial.tx(128)   'Address
  SabertoothSerial.tx(6)     'Command
  SabertoothSerial.tx(m_cmd) 'Data
  SabertoothSerial.tx((128 + 6 + m_cmd) & %01111111) 'Checksum

  'Drive _motor 2
  SabertoothSerial.tx(128)   'Address
  SabertoothSerial.tx(7)     'Command
  SabertoothSerial.tx(m_cmd) 'Data
  SabertoothSerial.tx((128 + 7 + m_cmd) & %01111111) 'Checksum


' The servo code is taken from http://obex.parallax.com/objects/51/
PRI SetServo(m_, m_pin, emergency) | m_cmd
  'm_ range value is -100 ~ 0 ~ 100
  if (m_ < -100)
    m_ := -100
  elseif (m_ > 100)
    m_ := 100

  ' emergency put the servo centered
  if (emergency == true)
    m_ := 0

  ' target value for the servo: 1000 ~ 1500 ~ 2000
  m_cmd := 1500 + (m_ * 5)
  Servo.Set(m_pin , m_cmd)
