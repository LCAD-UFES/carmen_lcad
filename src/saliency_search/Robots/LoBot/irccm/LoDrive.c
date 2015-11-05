/**
   \file  Robots/LoBot/irccm/LoDrive.c
   \brief Driving API for Robolocust iRobot Create Command Module control
   program.

   This file defines the functions that implement the driving API for a
   control program meant to be run on the iRobot Create's Command Module.
   These functions accept the high-level drive commands issued by the
   higher layers of the Robolocust controller and convert them to their
   equivalent Open Interface byte sequences.
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
   $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/irccm/LoDrive.c $
   $Id: LoDrive.c 13769 2010-08-08 01:34:02Z mviswana $
*/

/*------------------------------ HEADERS ------------------------------*/

// lobot headers
#include "LoDrive.h"
#include "LoSensors.h"
#include "LoUtils.h"
#include "LoIO.h"
#include "LoTimer.h"
#include "LoOpenInterface.h"

// avr headers
#include <avr/interrupt.h>

// Standard C headers
#include <stdlib.h>

/*------------------------------ GLOBALS ------------------------------*/

// The lobot controller's higher layers operate in terms of speed and
// steering directions. The iRobot Create's motion primitives, however,
// combine the speed and steering states into a single drive command
// (with special values to indicate driving straight ahead and turning
// in-place).
//
// Since the higher layers specify motor commands separately, we maintain
// two independent variables to keep track of the current speed and turn
// requests and update them as required based on the command sent in.
//
// Thus, when the high level issues turn commands, we update the turn
// radius but leave the speed variable alone. Similarly, when drive
// commands come in, we leave the turn radius as-is and update only the
// speed variable. But both variables get combined when we need to
// despatch the Open Interface drive command to the iRobot Create.
static volatile int g_speed ;
static volatile int g_radius = LOBOT_OI_DRIVE_STRAIGHT ;

// These variables are used to implement the acceleration/decelartion
// functionality asynchronously w.r.t. the main "thread."
//
// We need to have the speed ramping up/down operation performed
// asynchronously to ensure that the main "thread," which is responsible
// for listening and reacting to the robot's sensors, doesn't get caught
// up in a long loop that can cause it to miss a potentially dangerous
// situation (e.g., wheel drops or cliffs).
//
// We use the low-level controller's 10ms timer to achieve the desired
// asynchrony. Every time this timer fires, we ramp the speed up or down
// as required. As its name suggests, the g_target_speed variable is used
// to specify the desired speed to which the robot must ramp up or down.
// However, we need another variable (viz., g_ramp_flag) to indicate
// whether or not speed ramping should even occur. This is important
// because the drive module provides low-level functions that allow other
// parts of the low-level controller to bypass the
// acceleration/deceleration functionality (important, for instance, when
// reacting to wheel drops or cliffs).
static volatile char g_ramp_flag ;
static volatile int  g_target_speed ;

/*-------------------------- INITIALIZATION ---------------------------*/

// Forward declaration
static void accelerate_decelerate(void) ;

// This function initializes the drive system's acceleration/deceleration
// functionality by hooking into the 10ms timer's callback list.
void lo_init_drive(void)
{
   lo_add_timer10_cb(accelerate_decelerate) ;
}

/*--------------------- HIGH-LEVEL DRIVE COMMANDS ---------------------*/

// Sometimes, the high level may not have any commands queued. At those
// times, it should send a NOP to the low level. A NOP is exactly what
// it says: a no/null operation. If the high level doesn't send a NOP or
// sends some other unrecognized command (e.g., command code zero), the
// low-level controller will stop the robot, which may not be what is
// desired.
void lo_nop(int param){}

// Driving the robot forward
void lo_forward(int speed)
{
   lo_drive(lo_clamp(speed, 0, 500), g_radius, 1) ;
}

// Driving the robot backward
void lo_reverse(int speed)
{
   lo_drive(-lo_clamp(speed, 0, 500), g_radius, 1) ;
}

// Stopping the robot.
//
// DEVNOTE: The speed parameter is neither necessary nor utilized in this
// function. However, because the main module despatches incoming
// high-level commands using a map, we need to define all command
// handlers as taking one parameter.
void lo_stop(int speed)
{
   lo_drive(0, LOBOT_OI_DRIVE_STRAIGHT, 1) ;
}

// Turning the robot left
void lo_left(int turn_radius)
{
   if (turn_radius < 100)
      turn_radius = LOBOT_OI_TURN_INPLACE_CCW ;
   else
      turn_radius = lo_clamp(turn_radius, 100, 2000) ;
   lo_drive(g_speed, turn_radius, 0) ;
}

// Turning the robot right
void lo_right(int turn_radius)
{
   if (turn_radius < 100)
      turn_radius = LOBOT_OI_TURN_INPLACE_CW ;
   else
      turn_radius = -lo_clamp(turn_radius, 100, 2000) ;
   lo_drive(g_speed, turn_radius, 0) ;
}

// Straightening the robot.
//
// DEVNOTE: The turn_radius parameter is neither necessary nor utilized
// in this function. However, because the main module despatches incoming
// high-level commands using a map, we need to define all command
// handlers as taking one parameter.
void lo_straight(int turn_radius)
{
   lo_drive(g_speed, LOBOT_OI_DRIVE_STRAIGHT, 0) ;
}

// In-place turns
void lo_cmd_spin(int angle)
{
   // First, spin the robot by the requested amount and record the actual
   // amount of rotation.
   angle = lo_spin(225, lo_clamp(angle, -360, 360)) ;

   // Then, retrieve any unintended translational motion that occured
   // during the spin (can happen due to actuation errors).
   lo_wait(15) ;
   lo_tx(LOBOT_OI_CMD_QUERY_LIST) ;
   lo_tx(1) ;
   lo_tx(LOBOT_OI_SENSOR_DISTANCE) ;
   char buf[2] = {0} ;
   lo_rx(buf, 2, 20) ;

   // Finally, mark the actual amount of rotation and translation for the
   // next sensor packet so that high-level won't miss any crucial motion
   // updates.
   lo_update_pending_odometry(lo_make_word(buf[0], buf[1]), angle) ;
}

/*--------------------- LOW-LEVEL DRIVE COMMANDS ----------------------*/

// Helper function to send specified drive params in terms of Open
// Interface specs.
static void drive_oi(int speed, int turn)
{
   lo_tx(LOBOT_OI_CMD_DRIVE) ;
   lo_tx(lo_hibyte(speed)) ;
   lo_tx(lo_lobyte(speed)) ;
   lo_tx(lo_hibyte(turn))  ;
   lo_tx(lo_lobyte(turn))  ;
}

// This function takes care of accelerating or decelerating the robot
// asynchronously w.r.t. the main "thread" to ensure that the robot
// continues to listen and react to the sensors instead of getting tied
// up in a long loop that ramps the speed up or down. It is invoked by
// the 10ms timer's callback triggering mechanism.
static void accelerate_decelerate(void)
{
   if (g_ramp_flag)
   {
      if (g_speed < g_target_speed)
      {
         g_speed += 25 ;
         if (g_speed > g_target_speed)
            g_speed = g_target_speed ;
      }
      else if (g_speed > g_target_speed)
      {
         g_speed -= 25 ;
         if (g_speed < g_target_speed)
            g_speed = g_target_speed ;
      }
      else // target speed achieved
      {
         g_ramp_flag = 0 ;
         return ;
      }
      drive_oi(g_speed, g_radius) ;
   }
}

// Send Create drive commands in terms of speed and turn radius.
//
// DEVNOTE: To support the acceleration/deceleration functionality
// without tying up the main "thread" in a long loop that can make the
// robot deaf to its sensors while the speed ramps up/down, we use the
// ATmega168's Timer2 ISR (via the low-level controller's 10ms
// "generalized" timer) to perform the necessary speed ramping. The ISR
// uses a flag that is set or cleared by this function to determine
// whether or not perform the ramping operations.
//
// Additionally, the ISR uses and/or modifies some other variables
// that are also used in the main "thread" (e.g., g_speed).
//
// To ensure that the variables shared by the main thread and the Timer2
// ISR don't get mixed up, we need some sort of mutex mechanism.
// Unfortunately, avr-libc doesn't provide such a beast. And rolling our
// own is too much work. As a workaround, we simply disable the Timer2
// interrupt prior to modifying the shared variables and reenable it once
// we're done with the shared variables.
void lo_drive(int speed, int turn_radius, char smooth)
{
   if (g_speed == speed && g_radius == turn_radius)
      return ;

   g_radius = turn_radius ;
   if (smooth)
   {
      lo_suspend_timer10() ; // for atomicity, temporarily disable Timer2 int.
         g_ramp_flag = 1 ;
         g_target_speed = speed ;
      lo_resume_timer10()  ; // reenable Timer2 interrupt
   }
   else
   {
      lo_suspend_timer10() ; // for atomicity, temporarily disable Timer2 int.
         g_ramp_flag = 0 ;
      lo_resume_timer10()  ; // reenable Timer2 interrupt
      g_speed = speed ;
      drive_oi(g_speed, g_radius) ;
   }
}

// This function backs up the robot at the specified speed, moving it by
// an amount at least equal to the specified distance (in mm). It returns
// the actual distance backed up.
int lo_backup(int speed, int distance)
{
   // Start backing up
   lo_drive(speed, LOBOT_OI_DRIVE_STRAIGHT, 1) ;

   // Wait until the specified distance is reached
   int D = 0 ;
   while (abs(D) < abs(distance))
   {
      lo_wait(15) ;

      lo_tx(LOBOT_OI_CMD_QUERY_LIST) ;
      lo_tx(1) ;
      lo_tx(LOBOT_OI_SENSOR_DISTANCE) ;

      char buf[2] = {0} ;
      if (lo_rx(buf, 2, 20) >= 2)
         D += lo_make_word(buf[0], buf[1]) ;
   }
   lo_drive(0, LOBOT_OI_DRIVE_STRAIGHT, 1) ;
   return D ;
}

// Spin the robot at the specified speed by an amount equal to at least
// the specified angle. Return the actual amount spun.
int lo_spin(int speed, int angle)
{
   // Start spinning
   int turn = (angle < 0) ? LOBOT_OI_TURN_INPLACE_CW
                          : LOBOT_OI_TURN_INPLACE_CCW ;
   lo_drive(speed, turn, 0) ;

   // Wait until the specified angle is reached
   int A = 0 ;
   while (abs(A) < abs(angle))
   {
      lo_wait(15) ;

      lo_tx(LOBOT_OI_CMD_QUERY_LIST) ;
      lo_tx(1) ;
      lo_tx(LOBOT_OI_SENSOR_ANGLE) ;

      char buf[2] = {0} ;
      if (lo_rx(buf, 2, 20) >= 2)
         A += lo_make_word(buf[0], buf[1]) ;
   }
   lo_drive(0, LOBOT_OI_DRIVE_STRAIGHT, 0) ;
   return A ;
}

/*---------------------------- DRIVE STATE ----------------------------*/

// Is the robot stationary or moving?
char lo_stopped(void)
{
   return g_speed == 0 ;
}
