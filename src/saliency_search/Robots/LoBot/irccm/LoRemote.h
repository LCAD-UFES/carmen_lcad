/**
   \file  Robots/LoBot/irccm/LoRemote.h
   \brief Responding to the Roomba Remote.

   This file defines an API for checking whether the iRobot Create has
   received any IR commands from the Roomba Remote and taking appropriate
   action depending on the remote control button pressed.
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
   $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/irccm/LoRemote.h $
   $Id: LoRemote.h 13745 2010-08-02 05:27:16Z mviswana $
*/

#ifndef LOBOT_IRCCM_REMOTE_CONTROL_DOT_H
#define LOBOT_IRCCM_REMOTE_CONTROL_DOT_H

/*------------------------- LOW-LEVEL ACTIONS -------------------------*/

/// This function reacts to the Roomba Remote commands. If a remote
/// command was indeed sent by the user and handled by the low-level
/// controller, it sets a flag to indicate to the main program that a
/// remote control acknowledgement needs to be sent to the high level.
void lo_remote(void) ;

/*------------------ REMOTE CONTROL ACKNOWLEDGEMENTS ------------------*/

/// This function returns true if Robolocust's low-level Command Module
/// control program reacted to a Roomba Remote button press and needs to
/// send an acknowledgement to the high level letting it know what the
/// low level did.
char lo_remote_pending(void) ;

/// Send pending remote control acknowledgement to the high level.
void lo_send_remote(void) ;

/*------------------- REMOTE CONTROL STATE QUERIES --------------------*/

/// Returns true if the user is remote controlling the robot; false if
/// the robot is operating autonomously.
char lo_is_remote_control(void) ;

/// Returns true if the Roomba Remote's 'P' button has been pressed;
/// false otherwise.
///
/// NOTE: In addition to the 'P' button, the Roomba Remote's "Spot" and
/// "Max" buttons may also be used to quit the low-level controller.
char lo_remote_quit(void) ;

/*---------------------------------------------------------------------*/

#endif
