/**
   \file  Robots/LoBot/irccm/LoSensors.h
   \brief Sensors API for Robolocust iRobot Create Command Module control
   program.

   This file defines an API for retrieving relevant sensor data from the
   iRobot Create using the Open Interface.
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
   $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/irccm/LoSensors.h $
   $Id: LoSensors.h 13769 2010-08-08 01:34:02Z mviswana $
*/

#ifndef LOBOT_IRCCM_SENSORS_DOT_H
#define LOBOT_IRCCM_SENSORS_DOT_H

/*-------------------------- INITIALIZATION ---------------------------*/

/// This function initializes the sensors module. It must be called early
/// during the low-level controller's initialization sequence. This
/// function is responsible for properly debouncing the rear proximity
/// detectors.
void lo_init_sensors(void) ;

/// On its rear, the robot sports a pair of Sharp GP2D15 IR proximity
/// detectors to help prevent bumping into things as it backs up. Since
/// the Command Module is also accessed from the rear, these two sensors
/// detect the user's hand as soon as the robot is turned on and cause it
/// to spin and move forward, away from the "obstacle" behind it.
///
/// This can be very annoying, especially when we want the robot to
/// remain stationary at some fixed initial starting position prior to
/// the commencement of an experiment/run.
///
/// To work around this behaviour, we allow the high-level controller to
/// specify when the rear bumpers should be enabled and when they should
/// be ignored. Furthermore, the user can also toggle these sensors by
/// using the SPOT button on the Roomba Remote.
///
/// These functions turn the rear bumpers on/off.
//@{
void lo_enable_rear_bumps (int) ;
void lo_disable_rear_bumps(int) ;
void lo_toggle_rear_bumps(void) ;
//@}

/// When the rear bumpers are enabled, the low-level controller will
/// respond to rear bump events in one of two ways:
///
///    1. just stop the robot when either of the rear bump sensors is
///       active
///
///    2. stop the robot, then spin it a small amount either clockwise or
///       counterclockwise depending on which side the obstacle appears
///       and, finally, move the robot forward a little bit
///
/// By default, the low-level controller simply stops the robot. However,
/// the high-level controller can instruct the low-level controller to
/// adopt the second action described above by passing a '1' as the
/// parameter to the LOBOT_CMD_ENABLE_REAR_BUMPS command.
///
/// Additionally, the user can switch between the above two actions by
/// using the MAX button on the Roomba Remote.
///
/// This function toggles the flag responsible for deciding which of the
/// above two actions to take when a rear bump event is detected.
void lo_toggle_rear_bumps_spin(void) ;

/*---------------------- RETRIEVING SENSOR DATA -----------------------*/

/**
   This function retrieves the current sensor data from the Create
   robot. The Create is queried for all its sensor packets.

   To indicate to the main program that new sensor data has been
   retrieved, it sets a flag.

   The main loop of the low-level Command Module control program will
   test this flag in each iteration and then send the sensor data off to
   the high level.
*/
void lo_sensors(void) ;

/// The lo_sensors() function retrieves the current sensor data from the
/// Create robot and stores it in an internal array. This function
/// returns a sensor byte given its index.
///
/// NOTE: The indices into the internal sensor data array are defined in
/// LoCMInterface.h.
char lo_get_sensor(unsigned char index) ;

/*----------------------- UPDATING SENSOR DATA ------------------------*/

/// In response to events such as bumps and cliffs, the low-level
/// controller backs up the robot (or moves it forward if that is the
/// appropriate response) and spins it in place by some small amount.
/// When we do that, we should add the amount of linear and rotational
/// displacement to the encoder's estimates. Otherwise, the next sensor
/// packet we send to the high-level controller will report odometry
/// values associated with the sensor query sent to the iRobot Create
/// prior to the low-level controller's bumps/cliffs avoidance. These
/// incorrect odometry estimates can wreak havoc on any high-level
/// localization modules and other parts that rely on reasonably accurate
/// odometry from the low-level.
///
/// This function adds the given displacements to the low-level sensor
/// packet that is currently pending, i.e., "waiting" to be sent to the
/// high-level controller.
void lo_update_odometry(int distance, int angle) ;

/// When the high-level controller sends a SPIN command, it will specify
/// the amount to be spun. However, the actual amount spun may be
/// slightly more or less (due to motor/encoder errors). Furthermore, the
/// robot may also experience some slight translational motion in
/// addition to the requested rotation.
///
/// To ensure that the high-level receives accurate motion updates in the
/// next SENSORS acknowledgement, we should record the translation and
/// rotation resulting from the SPIN command in some temporary variables
/// and add those values to the encoder data retrieved from the Create
/// when the time comes to send the next ACK_SENSORS.
///
/// This function updates the temporary variables alluded to above.
void lo_update_pending_odometry(int distance, int angle) ;

/*------------------------- SENSOR DATA STATE -------------------------*/

/// Check if sensor data is available for sending to the high level.
char lo_sensors_pending(void) ;

/// When the rear bump sensors are active, the low-level controller can
/// be configured to respond by simply stopping the robot (default
/// behaviour) or by spinning in-place and then moving up. This function
/// returns false if the former configuration is in effect and true for
/// the latter.
char lo_rear_bumps_spin(void) ;

/*------------------------ SENDING SENSOR DATA ------------------------*/

/// Send pending/current sensor data to the high level.
void lo_send_sensors(void) ;

/*---------------------------------------------------------------------*/

#endif
