/**
   \file  Robots/LoBot/irccm/LoRemote.c
   \brief Implementation of the low-level remote control module.
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
   $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/irccm/LoRemote.c $
   $Id: LoRemote.c 13768 2010-08-07 23:41:41Z mviswana $
*/

/*------------------------------ HEADERS ------------------------------*/

// lobot headers
#include "LoDrive.h"
#include "LoSensors.h"
#include "LoIO.h"
#include "LoTimer.h"
#include "LoCMInterface.h"
#include "LoOpenInterface.h"

/*----------------------------- CONSTANTS -----------------------------*/

// The remote control module works by checking the robot's sensors to see
// if a remote control command has been sent. If so, it transitions to
// the REMOTE_CONTROL state and executes the specified command. While in
// the REMOTE_CONTROL state, if no more remote control commands come in
// within the predefined timeout, the module releases the motors and
// returns to the AUTONOMOUS state.
//
// This enumeration defines symbolic constants for the above-mentioned
// two states.
enum {
   AUTONOMOUS,
   REMOTE_CONTROL,
} ;

// Some parameters for this module
enum {
   LOBOT_REMOTE_DRIVE_SPEED = 250,
   LOBOT_REMOTE_TURN_SPEED  = 100,

   LOBOT_REMOTE_TIMEOUT = 1500,

   LOBOT_POWER_DEBOUNCE = 5,
} ;

/*------------------------------ GLOBALS ------------------------------*/

// This variable is used to keep track of the current state of the remote
// control module.
static char g_state ;

// As mentioned earlier, the remote control module works by keeping track
// of how long it has been since the most recent remote control command
// was received and reverting to normal autonomous operation on timeout.
// This variable is used to time-stamp the remote control commands.
static unsigned long g_time ;

// A data buffer for keeping track of pending ACK_REMOTE messages. The
// first byte of this buffer is used to store the ACK_REMOTE message
// itself; the remaining bytes are for the actual data that is sent with
// the acknowledgement.
static char g_remote[LOBOT_REMOTE_SIZE + 1] ;

// To help debounce the 'P' button press and ensure that user really does
// want to quit the lobot controller and dock the robot, we use a counter
// and return true for the "quit event" only if the counter has reached
// its minimum threshold (viz., LOBOT_POWER_DEBOUNCE).
static char g_power_count ;

/*------------------------- LOW-LEVEL ACTIONS -------------------------*/

// This helper function returns true if the specified IR byte corresponds
// to a supported remote control command; false otherwise.
static char supported_cmd(char c)
{
   switch (c)
   {
      case LOBOT_OI_REMOTE_LEFT:
      case LOBOT_OI_REMOTE_RIGHT:
      case LOBOT_OI_REMOTE_PAUSE:
      case LOBOT_OI_REMOTE_CLEAN:
      case LOBOT_OI_REMOTE_FORWARD:
      case LOBOT_OI_REMOTE_POWER:
      case LOBOT_OI_REMOTE_SPOT:
      case LOBOT_OI_REMOTE_MAX:
         return 1 ;
      default:
         return 0 ;
   }
}

// Helper function to respond to each of the remote control commands
static void execute(char ir_cmd)
{
   switch (ir_cmd)
   {
      case LOBOT_OI_REMOTE_FORWARD:
         lo_drive(LOBOT_REMOTE_DRIVE_SPEED, LOBOT_OI_DRIVE_STRAIGHT, 1) ;
         break ;
      case LOBOT_OI_REMOTE_CLEAN: // use clean button for driving backwards
         lo_drive(-LOBOT_REMOTE_DRIVE_SPEED, LOBOT_OI_DRIVE_STRAIGHT, 1) ;
         break ;
      case LOBOT_OI_REMOTE_PAUSE:
         lo_stop_immediate() ;
         break ;
      case LOBOT_OI_REMOTE_LEFT:
         lo_drive(LOBOT_REMOTE_TURN_SPEED, LOBOT_OI_TURN_INPLACE_CCW, 0) ;
         break ;
      case LOBOT_OI_REMOTE_RIGHT:
         lo_drive(LOBOT_REMOTE_TURN_SPEED, LOBOT_OI_TURN_INPLACE_CW, 0) ;
         break ;
      case LOBOT_OI_REMOTE_SPOT:
         lo_toggle_rear_bumps() ;
         break ;
      case LOBOT_OI_REMOTE_MAX:
         lo_toggle_rear_bumps_spin() ;
         break ;
      case LOBOT_OI_REMOTE_POWER:
         ++g_power_count ;
         return ; // important: no ACK_REMOTE when 'P' button is pressed
   }

   // Every time we execute a remote control command, we mark an
   // acknowledgement as pending.
   g_remote[0] = LOBOT_ACK_REMOTE ;
   g_remote[1] = ir_cmd ;
}

// The remote control module works by maintaining an internal state. When
// the user presses a supported remote control button, it transitions to
// the REMOTE_CONTROL state and takes over the robot's motors. When it
// times out waiting for a remote control command, it reverts to the
// AUTONOMOUS state and relinquishes the robot's motors so that normal
// operation may continue.
//
// DEVNOTE: This function does not check to see if sensor data is
// available before retrieving the infrared byte from the LoSensors
// module. The main program should take care of that, i.e., check that
// sensor data is actually available before calling this function. We do
// it like this because there are other low-level sensor reaction modules
// and all of them would have to keep checking the same flag over and
// over again. Much nicer if the main program checks the sensor data
// availability flag once and then calls all the sensor reaction
// functions one-by-one.
void lo_remote(void)
{
   char ir = lo_get_sensor(LOBOT_SENSORS_INFRARED_BYTE) ;
   switch (g_state)
   {
      case AUTONOMOUS:
         if (supported_cmd(ir))
         {
            execute(ir) ;
            g_state = REMOTE_CONTROL ;
            g_time  = lo_ticks() ;
         }
         break ;
      case REMOTE_CONTROL:
         if (lo_ticks() - g_time <= LOBOT_REMOTE_TIMEOUT)
         {
            if (supported_cmd(ir))
            {
               execute(ir) ;
               g_time = lo_ticks() ;
            }
         }
         else
         {
            g_state = AUTONOMOUS ;
            g_time  = 0 ;
            g_power_count = 0 ;
         }
         break ;
   }
}

/*------------------ REMOTE CONTROL ACKNOWLEDGEMENTS ------------------*/

// When the low level responds to remote control commands, it lets the
// high level know. This function checks if a ACK_REMOTE message is
// pending or not.
char lo_remote_pending(void)
{
   return g_remote[0] == LOBOT_ACK_REMOTE ;
}

// Send pending remote control acknowledgement to the high level
void lo_send_remote(void)
{
   for (unsigned char i = 0; i < LOBOT_REMOTE_SIZE + 1; ++i)
      lo_tx(g_remote[i]) ;

   // Sensor data no longer pending after being sent to high level
   g_remote[0] = 0 ;
}

/*------------------- REMOTE CONTROL STATE QUERIES --------------------*/

// Check if the user is remote controlling the robot or whether it is
// operating autonomously.
char lo_is_remote_control(void)
{
   return g_state == REMOTE_CONTROL ;
}

// Returns true if the Roomba Remote's 'P' button has been pressed;
// false otherwise.
char lo_remote_quit(void)
{
   return g_power_count >= LOBOT_POWER_DEBOUNCE ;
}
