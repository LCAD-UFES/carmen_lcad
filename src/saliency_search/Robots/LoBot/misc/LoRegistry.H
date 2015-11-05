/**
   \file Robots/LoBot/misc/LoRegistry.H

   \brief This file acts as a central repository for the names of
   different locust models, integartion algorithms, etc. supported by the
   Lobot/Robolocust project.
*/

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: mviswana usc edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/misc/LoRegistry.H $
// $Id: LoRegistry.H 14305 2010-12-08 21:17:33Z mviswana $
//

#ifndef LOBOT_REGISTRY_DOT_H
#define LOBOT_REGISTRY_DOT_H

//------------------------ LOCUST MODEL NAMES ---------------------------

#ifndef LOLM_GABBIANI
   #define LOLM_GABBIANI "gabbiani"
#endif

#ifndef LOLM_STAFFORD
   #define LOLM_STAFFORD "stafford"
#endif

//-------------------------- ROBOT PLATFORMS ----------------------------

// The Robolocust controller provides support for multiple robot
// platforms. Since each different platform will have a different set of
// sensorimotor primitives, we will need to create appropriate robot
// interface objects for controlling the robot's motors and retrieving
// its sensors. This section names the different robot platforms
// supported by Robolocust.

// The original Robolocust platform was a hacked R/C car with a mini-ITX
// Debian box, a Propeller board and a Sabertooth motor driver.
#ifndef LORP_LOBOT
   #define LORP_RC_CAR "rc_car"
#endif

// The Robolocust controller may also be run on an appropriately equipped
// iRobot Create/Roomba. The following name is for a Create/Roomba
// equipped with the iRobot Command Module.
#ifndef LORP_ROOMBA_CM
   #define LORP_ROOMBA_CM "roomba_cm"
#endif

//------------------------- ROBOT BEHAVIOURS ----------------------------

// This behaviour drives the robot forward at the configured cruising
// speed.
#ifndef LOBE_FORWARD
   #define LOBE_FORWARD "forward"
#endif

// This behaviour stops the robot when it senses things are too close for
// comfort.
#ifndef LOBE_EMERGENCY_STOP
   #define LOBE_EMERGENCY_STOP "emergency_stop"
#endif

// This behaviour gets the robot unstuck once the emergency stop
// behaviour has ground it to a halt.
#ifndef LOBE_EXTRICATE
   #define LOBE_EXTRICATE "extricate"
#endif

// This behaviour gets the robot unstuck by spinning the robot around to
// face the direction corresponding to minimal LGMD spiking activity.
#ifndef LOBE_LGMD_EXTRICATE_SIMPLE
   #define LOBE_LGMD_EXTRICATE_SIMPLE "lgmd_extricate_sim"
#endif

// This behaviour gets the robot unstuck by running the LGMD spikes
// through an array of elementary motion detectors.
#ifndef LOBE_LGMD_EXTRICATE_EMD
   #define LOBE_LGMD_EXTRICATE_EMD "lgmd_extricate_emd"
#endif

// This behaviour gets the robot unstuck by applying the virtual force
// field concept to the LGMD spikes rather than relying on the distance
// measurements made by the laser range finder.
#ifndef LOBE_LGMD_EXTRICATE_VFF
   #define LOBE_LGMD_EXTRICATE_VFF "lgmd_extricate_vff"
#endif

// This behaviour gets the robot unstuck by applying the virtual force
// field concept to distance "readings" derived from time-to-impact
// estimates based on LGMD spikes.
#ifndef LOBE_LGMD_EXTRICATE_TTI
   #define LOBE_LGMD_EXTRICATE_TTI "lgmd_extricate_tti"
#endif

// A purely computational, off-line task for computing the sensor model
// required by the lgmd_extricate_tti behaviour.
#ifndef LOBE_CALIBRATE_LET
   #define LOBE_CALIBRATE_LET "calibrate_lgmd_extricate_tti"
#endif

// This behaviour steers the robot towards the most open path that can
// accommodate the robot's width given the current distance measurements
// made by the laser range finder.
#ifndef LOBE_OPEN_PATH
   #define LOBE_OPEN_PATH "open_path"
#endif

// This behaviour implements a SLAM algorithm to obtain an occupancy map
// of the robot's surroundings and to localize the robot within that map.
#ifndef LOBE_SURVEY
   #define LOBE_SURVEY "survey"
#endif

// This behaviour uses the localization module to implement goal seeking.
#ifndef LOBE_GOAL
   #define LOBE_GOAL "goal"
#endif

// This behaviour uses the localization module to maintain a history of
// robot poses and periodically dump that to a file.
#ifndef LOBE_TRACK
   #define LOBE_TRACK "track"
#endif

// This behaviour simply logs bump events.
#ifndef LOBE_BUMP_COUNTER
   #define LOBE_BUMP_COUNTER "bump_counter"
#endif

// This behaviour monitors the danger zone and takes some user-specified
// action after the danger zone has been penetrated for some
// preconfigured duration.
#ifndef LOBE_MONITOR_DZONE
   #define LOBE_MONITOR_DZONE "monitor_danger_zone"
#endif

// This behaviour quits the high-level controller after some
// user-specified amount of time has elapsed since the robot started
// operating.
#ifndef LOBE_COUNTDOWN
   #define LOBE_COUNTDOWN "countdown"
#endif

// This behaviour collects various performance metrics and other useful
// data from other behaviours and periodically dumps this info to a file.
#ifndef LOBE_METRICS
   #define LOBE_METRICS "metrics"
#endif

// This behaviour renders the robot's trajectory and other such info
// collected in a results file (which is output by the lomet dataset
// analysis program) so we can take screenshots to include in papers,
// etc.
#ifndef LOBE_RENDER_RESULTS
   #define LOBE_RENDER_RESULTS "render_results"
#endif

// This behaviour tests Robolocust's laser range finder scan matching
// algorithm using some test data.
#ifndef LOBE_TEST_SCAN_MATCHING
   #define LOBE_TEST_SCAN_MATCHING "test_scan_matching"
#endif

// This behaviour uses the vector field histogram concept for guiding
// robot away from obstacles.
#ifndef LOBE_VFH
   #define LOBE_VFH "vfh"
#endif

// This behaviour can be used to manually control the robot.
//
// NOTE: It only works for the iRobot Roomba/Create, i.e., the
// "roomba_cm" platform.
#ifndef LOBE_REMOTE_CONTROL
   #define LOBE_REMOTE_CONTROL "remote_control"
#endif

//-----------------------------------------------------------------------

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
