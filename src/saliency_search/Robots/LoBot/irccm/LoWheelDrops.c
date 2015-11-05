/**
   \file  Robots/LoBot/irccm/LoWheelDrops.c
   \brief Low-level reactions for wheel drop sensors.

   This file defines the functions that implement the wheel drop sensor
   action and pending API for the low-level Robolocust control program
   meant to be run on the iRobot Create's Command Module.
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
   $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/irccm/LoWheelDrops.c $
   $Id: LoWheelDrops.c 13745 2010-08-02 05:27:16Z mviswana $
*/

/*------------------------------ HEADERS ------------------------------*/

// lobot headers
#include "LoWheelDrops.h"
#include "LoDrive.h"
#include "LoSensors.h"
#include "LoIO.h"
#include "LoCMInterface.h"
#include "LoOpenInterface.h"

/*------------------------------ GLOBALS ------------------------------*/

// This variable holds the current state of the wheel drop sensors as
// returned by the LoSensors module.
static char g_drop ;

/*--------------------- REACTING TO WHEEL DROPS -----------------------*/

// This function retrieves the wheel drop sensors' state from the
// LoSensors module and shuts off the motors in case any of them are
// active.
//
// DEVNOTE: This function does not check to see if sensor data is
// available before retrieving the wheel drop sensor state. The main
// program should take care of that, i.e., check that sensor data is
// actually available before calling this function. We do it like this
// because there are other low-level sensor reaction modules and all of
// them would have to keep checking the same flag over and over again.
// Much nicer if the main program checks the sensor data availability
// flag once and then calls all the sensor reaction functions one-by-one.
void lo_wheel_drops(void)
{
   g_drop = lo_get_sensor(LOBOT_SENSORS_WHEEL_DROPS) & LOBOT_OI_WHEEL_DROP_ANY;
   if (g_drop)
      lo_stop_immediate() ;
}

/*---------------- WHEEL DROP SENSOR ACKNOWLEDGEMENTS -----------------*/

// Check if wheel drops sensor acknowledgement is pending so that main
// loop can send out the ACK the next time it switches to talking to the
// high level via the Command Module's USB port.
char lo_wheel_drops_pending(void)
{
   return g_drop ;
}

// Send pending wheel drops sensor ACK to the high level
void lo_send_wheel_drops(void)
{
   lo_tx(LOBOT_ACK_WHEEL_DROPS) ;
   lo_tx(g_drop) ;
}
