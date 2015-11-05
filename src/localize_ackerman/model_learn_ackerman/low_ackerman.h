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
// low.h
//
// Copyright 2005, Austin Eliazar, Ronald Parr, Duke University
//
// low.c is where all of the actual localization and ancestry tree maintinence takes place.
// high.c is pretty much a direct copy of the code, geared towards performing the high level 
// in hierarchical SLAM.
//

#include <carmen/carmen.h>

#include "lowMap_ackerman.h"

// When using hierarchical SLAM, LOW_DURATION defines how many iterations of the low level are 
// performed to form one iteration of the high level. Setting this value to ridiculously high 
// values can essentially turn off the hierarchical feature.
#define LEARN_DURATION 100

// The function to call only once before LowSlam is called, and initializes all values.
void InitLowSlam();
// This function cleans up the memory and maps that were used by LowSlam.
void CloseLowSlam();
// The main function for performing SLAM at the low level. The first argument will return 
// whether there is still information to be processed by SLAM (set to 1). The second and third
// arguments return the corrected odometry for the time steps, and the corresponding list of
// observations. This can be used for the higher level SLAM process when using hierarchical SLAM.
void LowSlam(TPath **path, TSenseLog **obs);



extern double meanC_D, meanC_T, varC_D, varC_T, meanD_D, meanD_T, varD_D, varD_T, meanT_D, meanT_T, varT_D, varT_T;
extern carmen_FILE *readFile;
extern carmen_logfile_index_p logfile_index;
