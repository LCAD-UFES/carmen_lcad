/**
   \file  Robots/LoBot/irccm/LoDrive.h
   \brief Driving API for Robolocust iRobot Create Command Module control
   program.

   This file defines an API for driving the iRobot Create using the Open
   Interface. These functions accept the high-level drive commands issued
   by the higher layers of the Robolocust controller and convert them to
   their equivalent Open Interface byte sequences.
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
   $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/irccm/LoDrive.h $
   $Id: LoDrive.h 13769 2010-08-08 01:34:02Z mviswana $
*/

#ifndef LOBOT_IRCCM_DRIVE_DOT_H
#define LOBOT_IRCCM_DRIVE_DOT_H

/*-------------------------- INITIALIZATION ---------------------------*/

/// Initialize the drive system.
///
/// The main function must call lo_init_drive() during initialization to
/// ensure that the drive module's acceleration/deceleration
/// functionality is setup properly. Internally, acceleration and
/// deceleration are implemented using the ATmega168's Timer2 via the
/// low-level controller's 10ms "generalized timer" callback mechanism.
/// This function basically registers the callback function that
/// implements the acceleration/deceleration feature with the timer
/// module.
void lo_init_drive(void) ;

/*--------------------- HIGH-LEVEL DRIVE COMMANDS ---------------------*/

/// This command can be used by the high-level controller to have the
/// robot continue to do whatever it is currently doing. This is useful
/// when the high level doesn't have any real drive commands queued and
/// ready for the low level. Without this NOP, the low-level controller
/// will halt the robot if it doesn't receive a valid command within its
/// timeout window.
void lo_nop(int) ;

/// lobot's high-level controller works in terms of driving forwards,
/// backwards and stopping as distinct commands for the robot. These
/// functions implement the high-level drive commands using the generic
/// lo_drive function.
//@{
void lo_forward(int) ;
void lo_reverse(int) ;
void lo_stop(int) ;
//@}

/// lobot's high-level controller works in terms of steering the robot
/// using distinct left, right and straight commands. lobot also supports
/// in-place turns, i.e., spinning.
///
/// These functions implement the high-level steering commands using the
/// generic lo_drive function.
//@{
void lo_left(int turn_radius) ;
void lo_right(int turn_radius) ;
void lo_straight(int) ;
void lo_cmd_spin(int angle) ;
//@}

/*--------------------- LOW-LEVEL DRIVE COMMANDS ----------------------*/

/// This function implements a "general" drive command that takes a
/// speed and turn radius and converts it to the right set of Open
/// Interface byte sequences. Additionally, this function takes a flag
/// indicating whether the drive should be smooth (i.e., make use of the
/// acceleration/deceleration functionality implemented by this module)
/// or immediate (i.e., just implement the drive command without any
/// acceleration/deceleration).
void lo_drive(int speed, int turn_radius, char smooth) ;

/// This function executes an immediate stop.
///
/// The high-level drive commands go through an acceleration/deceleration
/// function to achieve the stated drive speed. However, when the
/// low-level controller senses a dangerous situation (e.g., wheel drops,
/// cliffs or bumps), it responds by taking appropriate action and then
/// informing the high level with a suitable ACK message. When responding
/// to low-level events, the low-level controller cannot afford to effect
/// smooth acceleration/decelaration because that might cause big
/// problems. Consider, for example, the robot approaching the edge of a
/// stairway. When the cliff sensors fire, the robot has to stop
/// immediately, pronto, right now, this very instant! If it were to
/// decelerate smoothly, it would just topple down the stairs...
///
/// This stop function allows low-level sensor reaction functions to
/// bypass the acceleration/deceleration functions that are used by the
/// high-level drive commands.
#define lo_stop_immediate() lo_drive(0, LOBOT_OI_DRIVE_STRAIGHT, 0)

/// This function backs up the robot by the specified amount at the
/// specified speed. It returns the actual amount backed up.
///
/// NOTE: Positive values for the backup speed and distance parameters
/// will result in moving the robot forward instead of backing it up.
int lo_backup(int speed, int distance) ;

/// This function spins the robot in-place by the specified angle at the
/// specified speed. It returns the actual amount spun.
///
/// Positive values for the spin angle result in ccw turns; negative
/// angles result in cw turns.
int lo_spin(int speed, int angle) ;

/*---------------------------- DRIVE STATE ----------------------------*/

/// Returns true if the robot is currently stopped; false if the robot is
/// currently moving.
char lo_stopped(void) ;

/// This function can be used to temporarily suspend the drive module's
/// acceleration/deceleration functionality. It works by inhibiting the
/// 10ms generalized timer's interrupts. It is required because the 10ms
/// timer is used to ramp the speed up or down as necessary and issue
/// Open Interface drive commands.
///
/// However, if the low-level controller is currently talking to the high
/// level via the Command Module's USB port, these drive commands will
/// get routed to the wrong serial destination. Thus, when the low-level
/// controller is busy interacting with the high level, it will need to
/// ensure that the drive module doesn't issue any drive commands related
/// to acceleration/deceleration.
#define lo_suspend_ramping() lo_suspend_timer10()

/// This function can be used to resume the drive module's temporarily
/// suspended acceleration/deceleration functionality. It works by
/// reenabling the 10ms generalized timer's interrupts. It is required
/// because the 10ms timer is used to ramp the speed up or down as
/// necessary and issue Open Interface drive commands.
///
/// However, if the low-level controller is currently talking to the high
/// level via the Command Module's USB port, these drive commands will
/// get routed to the wrong serial destination. Thus, when the low-level
/// controller is busy interacting with the high level, it will need to
/// ensure that the drive module doesn't issue any drive commands related
/// to acceleration/deceleration. Once the low level is done talking to
/// the high level, it will have to resume the speed ramping operations
/// to ensure that the robot moves smoothly instead of jerking about as
/// it starts and stops.
#define lo_resume_ramping() lo_resume_timer10()

/*---------------------------------------------------------------------*/

#endif
