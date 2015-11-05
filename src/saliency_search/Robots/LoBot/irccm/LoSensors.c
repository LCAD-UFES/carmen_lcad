/**
   \file  Robots/LoBot/irccm/LoSensors.c
   \brief Sensors API for Robolocust iRobot Create Command Module control
   program.

   This file defines the functions that implement the sensor retrieval
   API for the low-level Robolocust control program meant to be run on
   the iRobot Create's Command Module.
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
   $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/irccm/LoSensors.c $
   $Id: LoSensors.c 13798 2010-08-18 19:43:24Z mviswana $
*/

/*------------------------------ HEADERS ------------------------------*/

// lobot headers
#include "LoSensors.h"
#include "LoIO.h"
#include "LoTimer.h"
#include "LoUtils.h"
#include "LoCMInterface.h"
#include "LoOpenInterface.h"

// avr-libc headers
#include <avr/io.h>

/*----------------------------- CONSTANTS -----------------------------*/

enum {
   // lobot's rear bump sensors consist of a pair of Sharp GP2D15 IR
   // proximity detectors connected to the Command Module's cargo bay
   // ePort's digital I/O lines 1 and 2. These, in turn, connect to the
   // ATmega168's C register (bits 4 and 0 respectively).
   //
   // These bit masks are used to specify the PINC bits for checking the
   // bump sensor states.
   LOBOT_BUMP_RL_PIN = 0x10, // rear left  connected to C4
   LOBOT_BUMP_RR_PIN = 0x01, // rear right connected to C0

   // Instead of adding a new sensor data byte for the rear bumpers, we
   // simply use bits 5 and 6 of the bumps + wheel drops sensor data byte
   // (which are not used by the Open Interface).
   LOBOT_BUMP_RL_POS = 6, // rear left  bump sensor's bit position
   LOBOT_BUMP_RR_POS = 5, // rear right bump sensor's bit position

   // To debounce the rear bumpers, we expect to see a continuous logic
   // low for at least the following amount of time (in ms).
   LOBOT_REAR_BUMP_DEBOUNCE_THRESHOLD = 15,
} ;

/*------------------------------ GLOBALS ------------------------------*/

// We cannot retrieve the sensor data from the Create and directly send
// it off to the high level because the Command Module has a single USART
// that must be switched between talking to the robot's internal serial
// port and the Command Module's (external) USB port. Thus, when the
// high-level sends the command to retrieve sensor data, we store/buffer
// the data locally after getting it from the robot (via the Open
// Interface).
//
// Then, when the main loop of this low-level control program switches
// the USART to talk to the Robolocust computer, it will see if there is
// sensor data pending that needs to be sent to the high level. If so, it
// will perform the necessary sensor data send operation.
//
// The following variable is used to keep track of whether sensor data is
// pending or not.
static char g_sensors_pending ;

// The sensor data returned by the Create robot is buffered in this array
// prior to transmission to the high level.
static char g_sensors[LOBOT_SENSORS_SIZE] ;

// Due to motor/encoder errors, the actual amount of rotation in response
// to the SPIN command may be a little more or less than the requested
// amount. Furthermore, the robot may experience a slight translation in
// addition to the rotation. To ensure that the high-level controller
// does not miss out on any low-level motion updates, we have to add the
// actual amount of rotation and any unintended translation to the next
// sensor packet that will be sent to the high-level controller.
//
// These two variables temporarily record the values of the rotation and
// translation resulting from the SPIN command. They will be added to the
// encoder data for the next sensor packet.
static int g_pending_distance ;
static int g_pending_rotation ;

// On start-up, the rear bump sensors, a pair of Sharp GP2D15 IR
// proximity detectors, are ignored to prevent the robot from reacting to
// the user's hands as soon as the Command Module on/off switch is
// activated or when the user is trying to place the robot at some
// specific starting point before an experiment/run.
//
// This flag keeps track of whether the rear bumpers are enabled or not.
// The high-level controller can turn these two sensors on/off by using
// the appropriate commands.
static volatile char g_rear_bumps_enabled ;

// This flag indicates whether or not the robot should respond to rear
// bump events by spinning in-place and moving up a little or by simply
// stopping. The default behaviour is to just stop the robot. However,
// the high-level controller can instruct the low-level controller to
// spin and move the robot by passing 1 as the parameter to the
// LOBOT_CMD_ENABLE_REAR_BUMPS command.
static char g_rear_bumps_spin ;

// To debounce the rear proximity detectors, we use the following
// variables to count how many times the sensors are active in 1ms
// increments. If the count for a sensor exceeds the debounce threshold,
// only then is that detector considered active.
static volatile unsigned char g_rlb_count ; // rlb = rear left  bump
static volatile unsigned char g_rrb_count ; // rrb = rear right bump

/*-------------------------- INITIALIZATION ---------------------------*/

// Forward declaration
static void debounce_rear_bumps(void) ;

// Setup 1ms timer callback to perform debouncing asynchronously w.r.t.
// the main "thread." Otherwise, the main thread can get tied up in a
// reasonably long busy wait just to debounce the rear bumpers and
// potentially miss other important sensory input.
void lo_init_sensors(void)
{
   lo_add_timer1_cb(debounce_rear_bumps) ;
}

// Enable/disable the two rear bump sensors
void lo_enable_rear_bumps(int param)
{
   g_rear_bumps_enabled = 1 ;
   g_rear_bumps_spin = lo_lobyte(param) ;
}

void lo_disable_rear_bumps(int unused)
{
   g_rear_bumps_enabled = 0 ;
   g_rlb_count = g_rrb_count = 0 ;
}

void lo_toggle_rear_bumps(void)
{
   g_rear_bumps_enabled = ! g_rear_bumps_enabled ;
   g_rlb_count = g_rrb_count = 0 ;
}

// Enable/disable spin-and-move action for rear bump events
void lo_toggle_rear_bumps_spin(void)
{
   g_rear_bumps_spin = ! g_rear_bumps_spin ;
}

/*---------------------- RETRIEVING SENSOR DATA -----------------------*/

// Debouncing logic for the rear bump sensors: in 1ms increments, we
// check if the pins to which these sensors are connected report a logic
// high. If so, we increment the corresponding count variable for that
// sensor. If not, we reset the count to zero.
//
// Thus, when the main thread retrieves the counts, it will know
// immediately whether or not the rear bumpers were active for at least
// the last T milliseconds (where T is the debounce threshold).
//
// NOTE: It is possible for the main thread to check the counts when they
// are, say, halfway to the debounce threshold and then check again after
// several milliseconds. In the mean time, the 1ms timer might well have
// bumped the count past the threshold and, depending on the situation or
// stray noise, reset the counter. Thus, when the main thread checks
// next, it can potentially miss a crucial rear obstacle. Even worse,
// this could actually go on for a good, long time.
//
// The best way to avoid the situation described above would be to
// implement the debouncing using a Schmitt trigger rather than a simple
// count threshold. Nonetheless, in practice, this straightforward
// debouncing logic works fine.
static void debounce_rear_bumps(void)
{
   if (g_rear_bumps_enabled)
   {
      if (PINC & LOBOT_BUMP_RL_PIN) // rear left proximity detector active
         ++g_rlb_count ;
      else if (g_rlb_count > 0)
         g_rlb_count = 0 ;

      if (PINC & LOBOT_BUMP_RR_PIN) // rear right proximity detector active
         ++g_rrb_count ;
      else if (g_rrb_count > 0)
         g_rrb_count = 0 ;
   }
}

// Retrieve all the built-in sensors plus lobot's custom sensor states
void lo_sensors(void)
{
   // Send sensor query to the Create robot to return all sensor packets
   lo_tx(LOBOT_OI_CMD_SENSORS) ;
   lo_tx(LOBOT_OI_SENSOR_GROUP_6) ;

   // Retrieve sensor data from Create robot, waiting a maximum of 50ms
   // to get the data. Taking longer than 50ms to retrieve sensor data is
   // probably indicative of a temporary serial communication glitch with
   // the Create robot. We shouldn't keep waiting for the data as that
   // can hold up the robot in case the high level has detected a
   // problematic situation and issued appropriate commands to handle it.
   lo_rx(g_sensors, LOBOT_OI_SENSOR_SIZE_GROUP_6, 50) ;

   // Set the sensors pending flag, so that the low-level control
   // program's main loop can send the sensor data out to the high level.
   // But don't indicate to the main loop that sensor data is pending in
   // case the Create robot timed out and failed to respond to the above
   // sensor query.
   g_sensors_pending = ! lo_io_error() ;

   // When the high-level controller sends a SPIN command, the drive
   // module will mark the actual amount spun and any additional
   // translational motion as pending for the next SENSORS
   // acknowledgement. We need to add those amounts to the encoder data
   // returned by the above Open Interface exchange with the Create so
   // that the high-level doesn't miss any low-level motion updates and
   // end up getting mislocalized or experience some other related
   // problem.
   g_sensors[LOBOT_SENSORS_SPIN_FLAG] =
      g_sensors_pending && (g_pending_distance || g_pending_rotation) ;
   if (g_sensors[LOBOT_SENSORS_SPIN_FLAG]) {
      lo_update_odometry(g_pending_distance, g_pending_rotation) ;
      g_pending_distance = 0 ;
      g_pending_rotation = 0 ;
   }

   // Check the rear bump/proximity sensors connected to the Command
   // Module's cargo bay ePort.
   //
   // NOTE: Instead of adding a new byte of sensor data for the rear bump
   // sensors, we simply stuff these two bit flags into bits 5 and 6 of
   // the wheel drops + bumps sensor data byte.
   if (g_rlb_count >= LOBOT_REAR_BUMP_DEBOUNCE_THRESHOLD) {
      g_sensors[LOBOT_SENSORS_BUMPS] |= (1 << LOBOT_BUMP_RL_POS) ;
      g_rlb_count = 0 ;
   }
   if (g_rrb_count >= LOBOT_REAR_BUMP_DEBOUNCE_THRESHOLD) {
      g_sensors[LOBOT_SENSORS_BUMPS] |= (1 << LOBOT_BUMP_RR_POS) ;
      g_rrb_count = 0 ;
   }
}

// Return a sensor byte given its index.
char lo_get_sensor(unsigned char index)
{
   return g_sensors[index] ;
}

/*----------------------- UPDATING SENSOR DATA ------------------------*/

// Update odometry information stored in pending sensor packet (e.g., in
// response to bumps/cliffs action).
void lo_update_odometry(int distance, int angle)
{
   if (g_sensors_pending)
   {
      distance += lo_make_word(g_sensors[LOBOT_SENSORS_DISTANCE_HI],
                               g_sensors[LOBOT_SENSORS_DISTANCE_LO]) ;
      g_sensors[LOBOT_SENSORS_DISTANCE_HI] = lo_hibyte(distance) ;
      g_sensors[LOBOT_SENSORS_DISTANCE_LO] = lo_lobyte(distance) ;

      angle += lo_make_word(g_sensors[LOBOT_SENSORS_ANGLE_HI],
                            g_sensors[LOBOT_SENSORS_ANGLE_LO]) ;
      g_sensors[LOBOT_SENSORS_ANGLE_HI] = lo_hibyte(angle) ;
      g_sensors[LOBOT_SENSORS_ANGLE_LO] = lo_lobyte(angle) ;
   }
}

// Update odometry information for next sensor packet
void lo_update_pending_odometry(int distance, int angle)
{
   g_pending_distance += distance ;
   g_pending_rotation += angle ;
}

/*------------------------ SENSOR DATA STATE --------------------------*/

// Check if sensor data is pending so that main loop will send out the
// data the next time it switches to talking to the high level via the
// Command Module's USB port.
char lo_sensors_pending(void)
{
   return g_sensors_pending ;
}

// Check if rear bump action should be spin-and-move or just a plain stop
char lo_rear_bumps_spin(void)
{
   return g_rear_bumps_spin ;
}

/*------------------------ SENDING SENSOR DATA ------------------------*/

// Send pending sensor data to the high level
void lo_send_sensors(void)
{
   lo_tx(LOBOT_ACK_SENSORS) ;
   for (unsigned char i = 0; i < LOBOT_SENSORS_SIZE; ++i)
      lo_tx(g_sensors[i]) ;

   // Sensor data is no longer pending after it has been sent to the high
   // level.
   g_sensors_pending = 0 ;
}
