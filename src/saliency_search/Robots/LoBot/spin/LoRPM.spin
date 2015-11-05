{
   Robots/LoBot/spin/LoRPM.spin

   This file contains the code for a Propeller SPIN object that measures
   the RPM of one of lobot's wheels. LoMotorDriver.spin uses this object
   to return the current RPM to the higher-level C++ layers of the lobot
   controller, which can then regulate the robot's speed using this
   information.
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

'-------------------------- GLOBAL VARIABLES ----------------------------

VAR
   long stack [100]
   byte cog
   long RPMinput
   long RPMcount
   long timestamp
   long timeread

   'there are 4 value to count the rpm:
   long NoMagnet1
   long NoMagnet2
   long Magnet1
   long Magnet2

'------------------------------- OBJECTS --------------------------------

OBJ
   RPM1 : "LoServoInput"
   RPM2 : "LoServoInput"

'--------------------- INITIALIZATION AND CLEAN-UP ----------------------

PUB Start(pin,timeout) : okay
   Stop
   okay := cog := cognew(StartRPM(pin, timeout), @stack) + 1


PUB Stop
   if cog
      cogstop(cog~ - 1)

'-------------------------- PUBLIC INTERFACE ----------------------------

PUB GetRPM
   timestamp := cnt
   return (RPMcount/1000)


PUB GetMiliRPM
   timestamp := cnt
   return RPMcount

'------------------------- RUN-TIME INTERNALS --------------------------

DAT
   RPM_pins         LONG 0
   RPM1_pulseWidths LONG 1
   RPM2_pulseWidths LONG 1


PRI StartRPM(pin, timeout) | readvalue1, readvalue2, totaltime, data1, data2
   RPM_pins[0] := pin
   RPM1.start(@RPM_pins, 1, @RPM1_pulseWidths,  0)
   RPM2.start(@RPM_pins, 1, @RPM2_pulseWidths, -1)

   readvalue1 := 0
   readvalue2 := 0
   Magnet1    := 0
   Magnet2    := 0
   NoMagnet1  := 0
   NoMagnet2  := 0
   'timeread   := cnt

   repeat
      if cnt >= (timeread + clkfreq/1000*timeout)
         RPMcount := 0
      else
         totaltime := Magnet1 + Magnet2 + NoMagnet1 + NoMagnet2
         RPMcount  := 60_000_000 / (totaltime/1_000)

      data1 := RPM1_pulseWidths[0]
      data2 := RPM2_pulseWidths[0]
      if (data1 <> readvalue1 and data1 > 100) ' data1 new ==> low signal
         readvalue1 := data1
         timeread   := cnt
         Magnet2    := Magnet1
         Magnet1    := readvalue1
      if (data2 <> readvalue2 and data2 > 100) ' data2 new ==> high signal
         readvalue2 := data2
         timeread   := cnt
         NoMagnet2  := NoMagnet1
         NoMagnet1  := readvalue2

      'waitcnt (clkfreq/10 + cnt)

'-------------------- DEBUGGING/DEVELOPMENT SUPPORT ---------------------

PUB GetMagnet1
   return Magnet1

PUB GetMagnet2
   return Magnet2

PUB GetNoMagnet1
   return NoMagnet1

PUB GetNoMagnet2
   return NoMagnet2
