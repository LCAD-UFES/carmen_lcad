 /*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

/* /include/Nclient.h
 *
 * Nomadic client library
 * Public declarations
 *
 *                             ** COPYRIGHT NOTICE **
 *
 * Copyright (c) 1996, Nomadic Technologies, Inc.
 *
 * The contents of this  file is copyrighted by  Nomadic Technologies,
 * Inc, and is protected by United States copyright laws. You may make
 * as  many copies   of  the software   as  deemed  necessary  for the
 * effective programming of the Nomad  series robots provided that all
 * copyright and other proprietary   notices are reproduced  in  their
 * entirety.
 *
 */

#ifndef _N_INCLUDE_CLIENT_H_
#define _N_INCLUDE_CLIENT_H_

#include <limits.h>

/* --- Symbols ------------------------------------------------------------- */

/* General */

#define N_IGNORE                                Nan

#ifndef BOOL
#define BOOL unsigned char
#endif
#ifndef FALSE
#define FALSE ((BOOL)0)
#endif
#ifndef TRUE
#define TRUE  (!FALSE)
#endif

#define N_MAX_HOSTNAME_LENGTH   80

/* Measurement types for 'Unit' */
#define N_MILLIMETERS   'm'
#define N_SIXTEENTHS    'x'

/* Error codes for user functions. */
#define N_NO_ERROR                       0
#define N_UNKNOWN_ERROR                 -1
#define N_MISSING_ARGUMENT              -2
#define N_INVALID_ARGUMENT              -3
#define N_COMMAND_NOT_FOUND             -4
#define N_ROBOT_NOT_FOUND               -5
#define N_CONNECTION_FAILED             -6
#define N_ABNORMAL_ROBOT_POSITION       -7
#define N_TIMEOUT_ERROR                 -8
#define N_WRONG_OS_ERROR                -9
#define N_AXES_NOT_READY                -10
#define N_OUT_OF_MEMORY                 -11
#define N_UNSUPPORTED                   -12
#define N_SENSOR_NOT_READY              -13
#define N_DEVICE_NOT_FOUND              -14
#define N_UNINITIALIZED                 -15
#define CONNECTION_EXISTS                -16

/* Robot types, used in the RobotType field of the N_RobotState structure. */
#define N_INVALID_ROBOT_TYPE    '?'
#define N_N200_ROBOT_TYPE       'n'
#define N_XR4000_ROBOT_TYPE     'x'


/* --- Axes --- */

/* N_AxisSet.Status bit masks. */
#define N_AXES_READY         0
#define N_ESTOP_DOWN         1
#define N_JOYSTICK_IN_USE    2
#define N_MOTION_ERROR       4

/* Array bound for the Axis array field of the N_AxisSet structure. */
#define N_MAX_AXIS_COUNT     4

/* Indices into the Axis array field of the N_AxesSet structure for
 * XR4000 series robots. */
#define N_XTRANSLATION       0
#define N_YTRANSLATION       1
#define N_ROTATION           2

/* Axis modes. */
#define N_AXIS_NONE                  0
#define N_AXIS_ACCELERATION          1
#define N_AXIS_POSITION_RELATIVE     2
#define N_AXIS_POSITION_ABSOLUTE     3
#define N_AXIS_VELOCITY              4
#define N_AXIS_STOP                  5

/* --- Sonars --- */

/* Array bound for the SonarSet array field of the N_SonarController
 * structure. */
#define N_MAX_SONAR_SET_COUNT     6

/* Array bound for the Sonar array field of the N_SonarSet structure. */
#define N_MAX_SONAR_COUNT         16

/* Terminator for sonar set firing orders, used in the FiringOrder array field
 * of the N_SonarSet structure. */
#define N_END_SONAR_FIRING_ORDER  255

/* Value which indicates that a sonar timed out */
#define N_SONAR_TIMEOUT           LONG_MAX


/* --- Infrareds --- */

/* Array bound for the InfraredSet array field of the N_InfraredController
 * structure */
#define N_MAX_INFRARED_SET_COUNT  6

/* Array bound for the Infrared array field of the N_InfraredSet structure. */
#define N_MAX_INFRARED_COUNT      16

/* Terminator for infrared set firing orders, used in the FiringOrder array
 * field of the N_InfraredSet structure. */
#define N_END_INFRARED_FIRING_ORDER     255


/* --- Bumpers --- */

/* Array bound for the BumperSet array field of the N_BumperController
 * structure. */
#define N_MAX_BUMPER_SET_COUNT    6

/* Array bound for the Bumper array field of the N_BumperSet structure. */
#define N_MAX_BUMPER_COUNT        12

/* Possible bumper states, used in the Reading field of the N_Bumper
 * strucutre. */
#define N_BUMPER_NONE 0x0
#define N_BUMPER_LOW  0x1
#define N_BUMPER_HIGH 0x2

/* --- Batteries --- */

/* Array bound on the Battery field of the N_BatterySet structure. */
#define N_MAX_BATTERY_COUNT       4


/* Fields declared N_CONST are not meant to be modified by clients of this
 * API and will never be changed once initialized by N_ConnectRobot().  Do
 * not modify this definition. */
#ifndef N_CONST
#ifdef __cplusplus
#define N_CONST
#else
#define N_CONST 
#endif
#endif


/* --- Data structures ----------------------------------------------------- */


/* --- Integrated configuration --- */

struct N_Integrator
{
  BOOL DataActive;
  BOOL TimeStampActive;

  long x;
  long y;
  long Steering;
  long Rotation;
  unsigned long TimeStamp;
};

/* --- Axes --- */

struct N_Axis 
{
  BOOL DataActive;       /* Set to FALSE to make N_GetAxes ignore this axis. */
  BOOL TimeStampActive;  /* Set to TRUE to get time stamps with the data. */
  BOOL Update;           /* Set to TRUE to send this axis to the robot. */

  unsigned long TimeStamp;

  /* the Mode parameter describes how the settable parameters should be
   * interpreted when N_SetAxes is called, and is not retrieved by
   * N_GetAxes */
  char Mode;

  /* settable/retrievable parameters */
  long DesiredPosition;            /* Ignored for velocity moves. */
  long DesiredSpeed;               /* Scalar, greater than zero. */
  long Acceleration;               /* Scalar, greater than zero. */

  /* retrievable parameters */
  long TrajectoryPosition;         /* Instantaneous goal position. */
  long TrajectoryVelocity;         /* Instantaneous goal velocity. */
  long ActualPosition;             /* Current position. */
  long ActualVelocity;             /* Current velocity. */
  BOOL InProgress;                 /* FALSE if no move is being executed. */
};

struct N_AxisSet
{
  /* The global flag determines if the XR4000's X and Y axis use the global
   * reference frame (the x axis and y axis are determined by the robot's
   * orientation when it was zeroed), or the local reference frame (where
   * the y axis always points in the same direction as the front of the
   * robot).  This flag does not affect the IntegratedConfiguration
   * values.  Global mode is only supported by the XR4000. */
  BOOL Global;

  /* The status field can be used to check for unusual conditions in the
   * motor controller (e.g. e-stop down or joystick in use). */
  unsigned char Status;

  N_CONST unsigned int AxisCount;
  struct N_Axis Axis[N_MAX_AXIS_COUNT];
};

/* --- Joystick --- */

struct N_Joystick
{
  double X;
  double Y;
  double Theta;
  BOOL   ButtonA;
  BOOL   ButtonB;
  BOOL   ButtonC;
};

/* --- Sonars ---- */

struct N_Sonar
{
  long Reading;
  unsigned long TimeStamp;
};

struct N_SonarSet
{
  unsigned int FiringOrder[N_MAX_SONAR_COUNT + 1];
  long FiringDelay;
  long BlankingInterval;

  BOOL DataActive;
  BOOL TimeStampActive;
  N_CONST unsigned int SonarCount;
  struct N_Sonar Sonar[N_MAX_SONAR_COUNT];
};

struct N_SonarController
{
  N_CONST unsigned int SonarSetCount;
  struct N_SonarSet SonarSet[N_MAX_SONAR_SET_COUNT];
  BOOL SonarPaused;
};

/* --- Infrareds --- */

struct N_Infrared
{
  long Reading;
  unsigned long TimeStamp;
};

struct N_InfraredSet
{
  BOOL DataActive;
  BOOL TimeStampActive;
  N_CONST unsigned int InfraredCount;
  struct N_Infrared Infrared[N_MAX_INFRARED_COUNT];
};

struct N_InfraredController     
{
  BOOL InfraredPaused;
  N_CONST unsigned int InfraredSetCount;
  struct N_InfraredSet InfraredSet[N_MAX_INFRARED_SET_COUNT];
};

/* --- Bumpers --- */

struct N_Bumper
{
  char Reading;
  unsigned long TimeStamp;
};

struct N_BumperSet
{
  BOOL DataActive;
  BOOL TimeStampActive;
  N_CONST unsigned int BumperCount;
  struct N_Bumper Bumper[N_MAX_BUMPER_COUNT];
};

struct N_BumperController
{
  N_CONST unsigned int BumperSetCount;
  struct N_BumperSet BumperSet[N_MAX_BUMPER_SET_COUNT];
};

/* --- Batteries --- */

struct N_Battery
{
  long Voltage;
};

struct N_BatterySet
{
  N_CONST unsigned int BatteryCount;
  struct N_Battery Battery[N_MAX_BATTERY_COUNT];
  BOOL DataActive;
};

/* --- Timers --- */

struct N_Timer
{
  long Timeout;         /* Limp timeout value in msec. */
  unsigned long Time;   /* Current timestamp in msec. */
};


/* --- The main state structure -------------------------------------------- */

struct N_RobotState
{
  N_CONST long RobotID;
  N_CONST char RobotType;
  struct N_Integrator Integrator;
  struct N_AxisSet AxisSet;
  struct N_Joystick Joystick;
  struct N_SonarController SonarController;
  struct N_InfraredController InfraredController;
  struct N_BumperController BumperController; 
  struct N_BatterySet BatterySet; 
  struct N_Timer Timer;
};

typedef void (*N_ErrorFunc)(long RobotID);

/* --- User functions ------------------------------------------------------ */

#ifdef __cplusplus
extern "C"
{
#endif

/* Initialize the client library. */
int N_InitializeClient(const char *scheduler_hostname,
		       unsigned short scheduler_socket);

/* Connect and disconnect from a robot. */
int N_ConnectRobot(long RobotID);
int N_DisconnectRobot(long RobotID);

/* Retrieve a pointer to a robot's state structure. */
struct N_RobotState *N_GetRobotState(long RobotID);

/* Move the robot and retrieve axis configurations. */
int N_SetAxes(long RobotID);
int N_GetAxes(long RobotID);

/* Move the lift and retrieve lift axis configurations. */
int N_SetLift(long RobotID);
int N_GetLift(long RobotID);
int N_DeployLift(long RobotID);
int N_RetractLift(long RobotID);
int N_ZeroLift(long RobotID, BOOL Force);

/* Move the robot as though it were being controlled by a joystick. */
int N_SetJoystick(long RobotID);

/* Set and retrieve the current axis limp timeout value. */
int N_SetTimer(long RobotID);
int N_GetTimer(long RobotID);

/* Configure sonars and retrieve the current configuration; retrieve the
 * latest sonar readings. */
int N_SetSonarConfiguration(long RobotID);
int N_GetSonarConfiguration(long RobotID);
int N_GetSonar(long RobotID);

/* Configure infrareds and retrieve the current configuration; retrieve the
 * latest infrared readings. */
int N_SetInfraredConfiguration(long RobotID);
int N_GetInfraredConfiguration(long RobotID);
int N_GetInfrared(long RobotID);

/* Retrieve the latest bumper readings. */
int N_GetBumper(long RobotID);

/* Retrieve the latest battery state. */
int N_GetBattery(long RobotID);

/* Set and retrieve the current integrator values. */
int N_SetIntegratedConfiguration(long RobotID);
int N_GetIntegratedConfiguration(long RobotID);

/* Get all "DataActive" sensor readings at once. */
int N_GetState(long RobotID);

/* Send a text string to the speech system. */
int N_Speak(long RobotID, const char *Text);

#ifdef __cplusplus
}
#endif

#endif /* _N_INCLUDE_CLIENT_H_ */
