/**
   \file  Robots/LoBot/irccm/LoCliffs.c
   \brief Low-level reactions for cliff sensors.

   This file defines the functions that implement the cliff sensor action
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
   $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/irccm/LoCliffs.c $
   $Id: LoCliffs.c 13753 2010-08-03 15:59:25Z mviswana $
*/

/*------------------------------ HEADERS ------------------------------*/

// lobot headers
#include "LoCliffs.h"
#include "LoDrive.h"
#include "LoSensors.h"
#include "LoIO.h"
#include "LoUtils.h"
#include "LoCMInterface.h"
#include "LoOpenInterface.h"

/*------------------------------ GLOBALS ------------------------------*/

// The data that accompanies an ACK_CLIFFS message is stored in this
// array.
//
// DEVNOTE: Instead of using a separate variable to indicate that an
// ACK_CLIFFS is pending, we simply use the bit flags for the cliff
// sensors (stored in the first element of this array) as the pending
// flag. If none of the cliff sensors are active, it means no action was
// taken and, hence, no acknowledgement is pending.
static char g_cliffs[LOBOT_CLIFFS_SIZE] ;

/*------------------------ REACTING TO CLIFFS -------------------------*/

// This function retrieves the cliff sensors' states from the LoSensors
// module and backs up and spins the robot a little bit in case any of
// them are active.
//
// DEVNOTE: This function does not check to see if sensor data is
// available before retrieving the cliff sensors' states. The main
// program should take care of that, i.e., check that sensor data is
// actually available before calling this function. We do it like this
// because there are other low-level sensor reaction modules and all of
// them would have to keep checking the same flag over and over again.
// Much nicer if the main program checks the sensor data availability
// flag once and then calls all the sensor reaction functions one-by-one.
void lo_cliffs(void)
{
   g_cliffs[LOBOT_CLIFFS_FLAGS] =
      (lo_get_sensor(LOBOT_SENSORS_CLIFF_LEFT)        << 3) |
      (lo_get_sensor(LOBOT_SENSORS_CLIFF_FRONT_LEFT)  << 2) |
      (lo_get_sensor(LOBOT_SENSORS_CLIFF_FRONT_RIGHT) << 1) |
      (lo_get_sensor(LOBOT_SENSORS_CLIFF_RIGHT)) ;

   int backup = 0, spin = 0  ;
   switch (g_cliffs[LOBOT_CLIFFS_FLAGS])
   {
      case 0x0: // no cliffs
         break ;
      case 0x1: // right cliff
         backup = -100 ; spin = 15 ;
         break ;
      case 0x2: // front right cliff
         backup = -125 ; spin = 30 ;
      case 0x3: // front right and right cliffs
         backup = -150 ; spin = 45 ;
         break ;
      case 0x4: // front left cliff
         backup = -125 ; spin = -30 ;
         break ;
      case 0x5: // front left and right cliffs
         backup = -175 ; spin = 60 ;
         break ;
      case 0x6: // front left and front right cliffs
         backup = -200 ; spin = 90 ;
         break ;
      case 0x7: // front left and front right and right cliffs
         backup = -150 ; spin = 75 ;
         break ;
      case 0x8: // left cliff
         backup = -100 ; spin = -15 ;
         break ;
      case 0x9: // left and right cliffs
         backup = -100 ; spin = 180 ;
         break ;
      case 0xA: // left and front right cliffs
         backup = -175 ; spin = -60 ;
         break ;
      case 0xB: // left and front right and right cliffs
         backup = -125 ; spin = 90 ;
         break ;
      case 0xC: // left and front left cliffs
         backup = -150 ; spin = -45 ;
         break ;
      case 0xD: // left and front left and right cliffs
         backup = -175 ; spin = -120 ;
         break ;
      case 0xE: // left and front left and front right cliffs
         backup = -150 ; spin = -75 ;
         break ;
      case 0xF: // all cliffs
         backup = -300 ; spin = 180 ;
         break ;
   }

   if (backup) {
      backup = lo_backup(-225, backup) ;
      spin   = lo_spin(150, spin) ;
      lo_update_odometry(backup, spin) ;
   }

   g_cliffs[LOBOT_CLIFFS_DISTANCE_HI] = lo_hibyte(backup) ;
   g_cliffs[LOBOT_CLIFFS_DISTANCE_LO] = lo_lobyte(backup) ;
   g_cliffs[LOBOT_CLIFFS_ANGLE_HI]    = lo_hibyte(spin) ;
   g_cliffs[LOBOT_CLIFFS_ANGLE_LO]    = lo_lobyte(spin) ;
}

/*------------------- CLIFF SENSOR ACKNOWLEDGEMENTS -------------------*/

// Check if cliff sensor acknowledgement is pending so that main loop can
// send out the ACK the next time it switches to talking to the high
// level via the Command Module's USB port.
char lo_cliffs_pending(void)
{
   return g_cliffs[LOBOT_CLIFFS_FLAGS] ;
}

// Send pending cliff sensor ACK to high level
void lo_send_cliffs(void)
{
   lo_tx(LOBOT_ACK_CLIFFS) ;
   for (unsigned char i = 0; i < LOBOT_CLIFFS_SIZE; ++i)
      lo_tx(g_cliffs[i]) ;
}
