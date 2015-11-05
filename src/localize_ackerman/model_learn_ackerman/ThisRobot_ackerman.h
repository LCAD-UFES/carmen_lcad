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
// ThisRobot.h
//
// Copyright 2005, Austin Eliazar, Ronald Parr, Duke University
//
// ThisRobot is the interface between the code and the robot. 
// This contains information specific to the robot currently being used, as well as the functions which
// give commands or information requests to the robot.
//

#include "basic_ackerman.h"

// The number of sensor readings for this robot (typically 181 for a laser range finder)
#define SENSE_NUMBER 360
// Turn radius of the robot in map squares.Since the "robot" is actually the sensor origin for the
// purposes of this program, the turn radius is the displacement of the sensor from the robot's center
// of rotation (assuming holonomic turns)
#define TURN_RADIUS (0.72 * MAP_SCALE)

// Each sensor reading has direction it is looking (theta) and a distance at which it senses the object.
// Direction of the sensor is relative to the facing of the robot, with "forward" being 0, and is
// measured in radians. 
struct TSense_struct{
  double theta, distance;
};
typedef struct TSense_struct TSenseSample;
typedef TSenseSample TSense[SENSE_NUMBER+1];

// This is the structure for storing odometry data from the robot. The same conditions apply as above.
struct odo_struct{
  double x, y, theta;
};
typedef struct odo_struct TOdo;

extern TOdo odometry;

