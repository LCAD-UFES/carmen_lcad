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
// basic.h
//
// Copyright 2005, Austin Eliazar, Ronald Parr, Duke University
//
// basic stuff used by everybody 
//

// These includes pull in interface definitions and utilities.
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

// This defines the number of grid squares per meter. 
// All sensors are assumed to be measuring in meters, whereas all code uses a basic unit of grid squares. 
// We use this constant for conversion.
#define MAP_SCALE 10

// Some useful macros
#define SIGN(A) ((A) >= 0.0 ? (1.0) : (-1.0))
#define SQUARE(A) (((A) * (A)))

// An approximation of a normal distribution, where A is the standard deviation
#define GAUSSIAN(A) ((MTrandDec()+MTrandDec()+MTrandDec()+MTrandDec()+MTrandDec()+MTrandDec()-3)*(A)/.7071)

#define FALSE 0
#define TRUE 1

// When recording a log file, the file name of the log is stored in *RECORDING
// When playing back a log file, the name is stored in *PLAYBACK
extern char *PLAYBACK, *RECORDING;
