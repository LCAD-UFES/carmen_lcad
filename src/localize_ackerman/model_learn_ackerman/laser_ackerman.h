// 
// This Program is provided by Duke University and the authors as a service to the
// research community. It is provided without cost or restrictions, except for the
// User's acknowledgement that the Program is provided on an "As Is" basis and User
// understands that Duke University and the authors make no express or implied
// warranty of any kind.  Duke University and the authors specifically disclaim any
// implied warranty or merchantability or fitness for a particular purpose, and make
// no representations or warranties that the Program will not infringe the
// intellectual property rights of others. The User agrees to indemnify and hold
// harmless Duke University and the authors from and against any and all liability
// arising out of User's use of the Program.
//
// laser.h
//
// Copyright 2005, Austin Eliazar, Ronald Parr, Duke University
//
// This file contains definitions relating to the laser
//

#include "ThisRobot_ackerman.h"

// Variance of the laser measurements in Grid Squares
// These numbers are significantly larger than the actual variance
// that the laser readings give, as they represent a large number of 
// different variables, all of which are wrapped up into error in the
// laser reading (ie elevation, tilt, rotational calibration, uneven 
// surfaces, particle densities, etc.)
// Note that the variance for the high level in hierarchical slam is 
// much higher. This is due largely to the fact that the high level 
// takes into account so many more sensor readings, and must also 
// account for possible driftwithin a single iteration
#define LOW_VARIANCE (0.17 * MAP_SCALE*MAP_SCALE)
#define HIGH_VARIANCE (0.017 * MAP_SCALE*MAP_SCALE)

// Set the maximum usuable distance for the laser range finder. This number is often less than the actual
// reliable distance for the specific LRF, because the laser casts 'scatter' at long distances.
#define MAX_SENSE_RANGE 10.0 * MAP_SCALE

