/**
   \file  Robots/LoBot/irccm/LoBumps.c
   \brief Low-level reactions for bump sensors.

   This file defines the functions that implement the bump sensor action
   and pending API for the low-level Robolocust control program meant to
   be run on the iRobot Create's Command Module.
*/

/*
 ************************************************************************
 * The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   *
 * by the University of Southern California (USC) and the iLab at USC.  *
 * See http://iLab.usc.edu for information about this project.          *
 *                                                                      *
 * Major portions of the iLab Neuromorphic Vision Toolkit are protected *
 * under the U.S. patent ``Computation of Intrinsic Perceptual Saliency *
 * in Visual Environments, and Applications'' by Christof Koch and      *
 * Laurent Itti, California Institute of Technology, 2001 (patent       *
 * pending; application number 09/912,225 filed July 23, 2001; see      *
 * http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     *
 ************************************************************************
 * This file is part of the iLab Neuromorphic Vision C++ Toolkit.       *
 *                                                                      *
 * The iLab Neuromorphic Vision C++ Toolkit is free software; you can   *
 * redistribute it and/or modify it under the terms of the GNU General  *
 * Public License as published by the Free Software Foundation; either  *
 * version 2 of the License, or (at your option) any later version.     *
 *                                                                      *
 * The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  *
 * that it will be useful, but WITHOUT ANY WARRANTY; without even the   *
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      *
 * PURPOSE.  See the GNU General Public License for more details.       *
 *                                                                      *
 * You should have received a copy of the GNU General Public License    *
 * along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   *
 * to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   *
 * Boston, MA 02111-1307 USA.                                           *
 ************************************************************************
*/

/*
   Primary maintainer for this file: mviswana usc edu
   $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/irccm/LoBumps.c $
   $Id: LoBumps.c 13768 2010-08-07 23:41:41Z mviswana $
*/

/*------------------------------ HEADERS ------------------------------*/

// lobot headers
#include "LoBumps.h"
#include "LoDrive.h"
#include "LoSensors.h"
#include "LoIO.h"
#include "LoUtils.h"
#include "LoCMInterface.h"
#include "LoOpenInterface.h"

/*------------------------------ GLOBALS ------------------------------*/

// The data that accompanies an ACK_BUMPS message is stored in this
// array.
//
// DEVNOTE: Instead of using a separate variable to indicate that an
// ACK_BUMPS is pending, we simply use the bit flags for the bump sensors
// (stored in the first element of this array) as the pending flag. If
// none of the bump sensors was active, it means no action was taken and,
// hence, no acknowledgement is pending.
static char g_bumps[LOBOT_BUMPS_SIZE] ;

/*--------------------- REACTING TO BUMP SENSORS ----------------------*/

// This function retrieves the bump sensor state from the LoSensors
// module and takes appropriate action.
//
// DEVNOTE: This function does not check to see if sensor data is
// available before retrieving the bump sensor state. The main program
// should take care of that, i.e., check that sensor data is actually
// available before calling this function. We do it like this because
// there are other low-level sensor reaction modules and all of them
// would have to keep checking the same flag over and over again. Much
// nicer if the main program checks the sensor data availability flag
// once and then calls all the sensor reaction functions one-by-one.
void lo_bumps(void)
{
   char bumps = lo_get_sensor(LOBOT_SENSORS_BUMPS) ;
   g_bumps[LOBOT_BUMPS_FLAGS] =
       (bumps & LOBOT_OI_BUMP_BOTH) |         // front bumpers
      ((bumps & LOBOT_BUMP_REAR_BOTH) >> 3) ; // rear bumpers

   bumps = g_bumps[LOBOT_BUMPS_FLAGS] ;

   int backup = 0, spin = 0 ;
   if ((bumps & LOBOT_OI_BUMP_BOTH) == LOBOT_OI_BUMP_BOTH)
   {
      backup = -300 ;
      spin   = -30  ;
   }
   else if (bumps & LOBOT_OI_BUMP_LEFT)
   {
      backup = -150 ;
      spin   =  -15 ;
   }
   else if (bumps & LOBOT_OI_BUMP_RIGHT)
   {
      backup = -150 ;
      spin   =   15 ;
   }
   else if ((bumps & LOBOT_BUMP_ACK_REAR_BOTH) == LOBOT_BUMP_ACK_REAR_BOTH)
   {
      backup =  25 ; // NOTE: +ve backup for rear bumper ==> move forward
      spin   =  20 ;
   }
   else if (bumps & LOBOT_BUMP_ACK_REAR_LEFT)
   {
      backup =  75 ; // NOTE: +ve backup for rear bumper ==> move forward
      spin   =  10 ;
   }
   else if (bumps & LOBOT_BUMP_ACK_REAR_RIGHT)
   {
      backup =  75 ; // NOTE: +ve backup for rear bumper ==> move forward
      spin   = -10 ;
   }

   if (bumps & LOBOT_OI_BUMP_EITHER) // front bump ==> first backup, then spin
   {
      backup = lo_backup(-225, backup) ;
      spin   = lo_spin(150, spin) ;
      lo_update_odometry(backup, spin) ;
   }
   else if (bumps & LOBOT_BUMP_ACK_REAR_EITHER) // rear bump
   {
      if (lo_rear_bumps_spin()) // configured for spinning and moving up
      {
         spin   = lo_spin(150, spin) ;
         backup = lo_backup(225, backup) ;
         lo_update_odometry(backup, spin) ;
      }
      else // rear bump configured only to stop
         lo_stop_immediate() ;
   }

   g_bumps[LOBOT_BUMPS_DISTANCE_HI] = lo_hibyte(backup) ;
   g_bumps[LOBOT_BUMPS_DISTANCE_LO] = lo_lobyte(backup) ;
   g_bumps[LOBOT_BUMPS_ANGLE_HI]    = lo_hibyte(spin) ;
   g_bumps[LOBOT_BUMPS_ANGLE_LO]    = lo_lobyte(spin) ;
}

/*------------------- BUMP SENSOR ACKNOWLEDGEMENTS --------------------*/

// Check if bump sensor acknowledgement is pending so that main loop can
// send out the ACK the next time it switches to talking to the high
// level via the Command Module's USB port.
char lo_bumps_pending(void)
{
   return g_bumps[LOBOT_BUMPS_FLAGS] ;
}

// Send pending bump sensor ACK to the high level
void lo_send_bumps(void)
{
   lo_tx(LOBOT_ACK_BUMPS) ;
   for (unsigned char i = 0; i < LOBOT_BUMPS_SIZE; ++i)
      lo_tx(g_bumps[i]) ;

   // Sensor data is no longer pending after it has been sent to the high
   // level.
   g_bumps[LOBOT_BUMPS_FLAGS] = 0 ;
}
