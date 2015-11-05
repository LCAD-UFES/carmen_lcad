/**
   \file  Robots/LoBot/irccm/LoWheelDrops.h
   \brief Low-level reactions for the wheel drop sensors.

   This file defines an API for examining the iRobot Create's wheel drop
   sensor states and stopping lobot's motors in case it has teetered off
   the edge of a stairway or been picked up. Additionally, this module
   provides an API for sending the wheel drops acknowledgement to the
   high level.
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
   $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/irccm/LoWheelDrops.h $
   $Id: LoWheelDrops.h 13745 2010-08-02 05:27:16Z mviswana $
*/

#ifndef LOBOT_IRCCM_WHEEL_DROPS_DOT_H
#define LOBOT_IRCCM_WHEEL_DROPS_DOT_H

/*------------------------- LOW-LEVEL ACTIONS -------------------------*/

/// This function reacts to the iRobot Create's wheel drop sensors. If
/// any of them are indeed active, it sets a flag to indicate to the main
/// program that a wheel drops acknowledgement needs to be sent to the
/// high level.
void lo_wheel_drops(void) ;

/*---------------- WHEEL DROP SENSOR ACKNOWLEDGEMENTS -----------------*/

/// This function returns true if Robolocust's low-level Command Module
/// control program reacted to the Create robot's wheel drop sensors and
/// needs to send an acknowledgement to the high level letting it know
/// what the low level did.
char lo_wheel_drops_pending(void) ;

/// Send pending wheel drops acknowledgement to the high level.
void lo_send_wheel_drops(void) ;

/*--------------------- WHEEL DROP STATE QUERIES ----------------------*/

/// Returns true if the wheel drop module's internal state indicates that
/// a wheel is dropped; false otherwise.
#define lo_is_wheel_dropped() lo_wheel_drops_pending()

/*---------------------------------------------------------------------*/

#endif
