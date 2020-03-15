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

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <string.h>
#include "extend.h"
#include "i200m.h"
#include "Nclient.h"
#include "setup.h"
#include "arcnet.h"
#include "arcnet_sensors.h"

struct I100_sensor_data {
  long int reading;
  long int auxiliary;
  long unsigned int sens_time;
  long int integ_x;
  long int integ_y;
  long int integ_steer;
  long int integ_turret;
  long int vel_trans;
  long int vel_steer;
  long int vel_turret;
  long unsigned int integ_time;
};

static unsigned char rclnt_initialized = 0; 
static unsigned char rclnt_connected = 0;

typedef enum {
  RCLNT_I100_BATTERY, 
  RCLNT_PDB_BATTERY
} RCLNT_BatteryType;

static long int rclnt_robot_id; 
static char rclnt_robot_type; 
static RCLNT_BatteryType rclnt_battery_type; 
static int rclnt_battery_fd; 
static int Infrared; 
static int Sonar; 
static int Bumper; 
static int Zero; 
static int Sound; 
static int Clock; 
static double RobotTimeout; 

struct N_RobotState rclnt_rs; 
struct N_RobotStateExt rclnt_rs_ext; 

int N_InitializeClient(const char *scheduler_hostname
		       __attribute__ ((unused)), 
		       unsigned short scheduler_socket
		       __attribute__ ((unused)))
{
  char *setup_value; 

  if (rclnt_initialized == 1)
    return 0;
  
  if ((setup_value = SETUP_GetValue("[robot]type")) == NULL) {
    rclnt_rs.RobotType = 'x';
    rclnt_robot_type = 'x';
  } else if ((strcasecmp(setup_value, "x") == 0) ||
	     (strcasecmp(setup_value, "xr4k") == 0) ||
	     (strcasecmp(setup_value, "xr4000") == 0)) {
    rclnt_rs.RobotType = 'x';
    rclnt_robot_type = 'x';
  } else if ((strcasecmp(setup_value, "n") == 0) ||
	     (strcasecmp(setup_value, "n200") == 0) ||
	     (strcasecmp(setup_value, "nomad200") == 0)) {
    rclnt_rs.RobotType = 'n';
    rclnt_robot_type = 'n';
  } else {
    fprintf(stderr, "Nrobot: (fatal) %s is not a valid value for %s\n"
	    "This entry is required to determine the appropriate "
	    "hardware interface to\n"
	    "use.  For further information, please consult your manual.\n",
	    setup_value, "[robot]type");
    return N_CONNECTION_FAILED;
  } 

  Sound = open("/dev/dbtk", 1);
  
  switch (rclnt_robot_type) {
  case 'n':
    Sonar = open("/dev/i100_sonar", 2);
    Bumper = open("/dev/i100_bumper", 0);
    Infrared = open("/dev/i100_infrared", 2);
    Zero = open("/dev/i100_zeroswitch", 0);
    Clock = open("/dev/i100_clock", 0);
    I200M_Initialize(&rclnt_rs);
    rclnt_rs.AxisSet.AxisCount = 3;
    rclnt_battery_type = RCLNT_I100_BATTERY;
    rclnt_battery_fd = open("/dev/i100_battery", 0);
    rclnt_rs.BatterySet.BatteryCount = 2;
    break;

  case 'x': 
    ANET_Initialize();
    I200M_Initialize(&rclnt_rs);
    rclnt_rs.AxisSet.AxisCount = 3;
    rclnt_battery_type = RCLNT_PDB_BATTERY;
    rclnt_rs.BatterySet.BatteryCount = 4;
    break;
  }
  
  SON_Initialize(&rclnt_rs, &rclnt_rs_ext);
  INF_Initialize(&rclnt_rs, &rclnt_rs_ext);
  BUMP_Initialize(&rclnt_rs, &rclnt_rs_ext);
  
  rclnt_initialized = 1;
  
  return 0;
}

int N_PollClient()
{
  ANET_Poll();
  return 0;
}

int N_ConnectRobot(long int robot_id)
{
  int i; 

  if (!rclnt_initialized)
    return N_CONNECTION_FAILED;
  
  if (rclnt_connected) {
    if (robot_id == rclnt_robot_id)
      return 0;
    else 
      return N_ROBOT_NOT_FOUND;
  }
  
  rclnt_rs.RobotID = rclnt_robot_id = robot_id;

  for (i = 0; i < (int)rclnt_rs.AxisSet.AxisCount; i++) {
    rclnt_rs.AxisSet.Axis[i].Update = 0;
    rclnt_rs.AxisSet.Axis[i].DataActive = 1;
    rclnt_rs.AxisSet.Axis[i].TimeStampActive = 0;    
  }
  
  for (i = 0; i < (int)rclnt_rs.SonarController.SonarSetCount; i++) {
    rclnt_rs.SonarController.SonarSet[i].DataActive = 1;
    rclnt_rs.SonarController.SonarSet[i].TimeStampActive = 0;
  }

  for (i = 0; i < (int)rclnt_rs.InfraredController.InfraredSetCount; i++) {
    rclnt_rs.InfraredController.InfraredSet[i].DataActive = 1;
    rclnt_rs.InfraredController.InfraredSet[i].TimeStampActive = 0;
  }

  for (i = 0; i < (int)rclnt_rs.BumperController.BumperSetCount; i++) {
    rclnt_rs.BumperController.BumperSet[i].DataActive = 1;
    rclnt_rs.BumperController.BumperSet[i].TimeStampActive = 0;
  }

  rclnt_rs.Integrator.DataActive = 1;
  rclnt_rs.Integrator.TimeStampActive = 0;
  rclnt_rs.BatterySet.DataActive = 1;
  
  rclnt_connected = 1;
  
  return 0;
}

int N_DisconnectRobot(long int robot_id)
{
  if ((rclnt_connected == 0) || (robot_id != rclnt_robot_id))
    return N_ROBOT_NOT_FOUND;

  rclnt_connected = 0;
  return 0;
}

struct N_RobotState *N_GetRobotState(long int robot_id)
{
  if ((rclnt_connected == 0) || (robot_id != rclnt_robot_id))
    return NULL;
  
  return &rclnt_rs;
}

int N_GetInfrared(long int robot_id)
{
  int i; 
  struct I100_sensor_data SensorData[16]; 

  if ((rclnt_connected == 0) || (robot_id != rclnt_robot_id))
    return N_ROBOT_NOT_FOUND;
  
  switch (rclnt_robot_type) {
  case 'n': 
    if (rclnt_rs.InfraredController.InfraredSet[0].DataActive == 0)
      return 0;

    if (lseek(Infrared, 0, SEEK_SET) < 0)
      return N_UNKNOWN_ERROR;
    
    if (read(Infrared, &SensorData, sizeof(SensorData)) < 0)
      return N_UNKNOWN_ERROR;
    
    for (i = 0; i < 16; i++) {
      rclnt_rs.InfraredController.InfraredSet[0].Infrared[i].Reading =
	SensorData[i].reading / 2400000.0 * 255.0;

      if (rclnt_rs.InfraredController.InfraredSet[0].TimeStampActive)
	rclnt_rs.InfraredController.InfraredSet[0].Infrared[i].TimeStamp =
	  SensorData[i].sens_time;
    }
    
    break;
    
  case 'x': 
    break;
    
  }
  return 0;
}

int N_SetSonarConfiguration(long int robot_id)
{
  if ((rclnt_connected == 0) || (robot_id != rclnt_robot_id))
    return N_ROBOT_NOT_FOUND;
  
  switch (rclnt_robot_type) {
  case 'n': 
    
    if (ioctl(Sonar, 0, &(rclnt_rs.SonarController.SonarSet[0].FiringDelay))
	< 0)
      return N_UNKNOWN_ERROR;

    if (ioctl(Sonar, 2, &(rclnt_rs.SonarController.SonarSet[0].FiringOrder))
	< 0)
      return N_UNKNOWN_ERROR;
    
    break;

  case 'x': 
    ANET_ConfSonar();
    break;
    
  }
  
  return 0;
}

int N_GetSonarConfiguration(long int robot_id)
{
  if ((rclnt_connected == 0) || (robot_id != rclnt_robot_id))
    return N_ROBOT_NOT_FOUND;

  switch (rclnt_robot_type) {
  case 'n': 
    if (ioctl(Sonar, 1, &(rclnt_rs.SonarController.SonarSet[0].FiringDelay)) 
	< 0)
      return N_UNKNOWN_ERROR;

    if (ioctl(Sonar, 3, &(rclnt_rs.SonarController.SonarSet[0].FiringOrder))
	< 0)
      return N_UNKNOWN_ERROR;
    
    break;
    
  case 'x': 
    break;
  }
  
  return 0;
}

int N_GetSonar(long int robot_id)
{
  int i; 
  struct I100_sensor_data SensorData[16]; 

  if ((rclnt_connected == 0) || (robot_id != rclnt_robot_id))
    return N_ROBOT_NOT_FOUND;

  switch (rclnt_robot_type) {
  case 'n': 
    if (!rclnt_rs.SonarController.SonarSet[0].DataActive)
      return 0;

    if (lseek(Sonar, 0, SEEK_SET) < 0)
      return N_UNKNOWN_ERROR;
    
    if (read(Sonar, &SensorData, sizeof(SensorData)) < 0)
      return N_UNKNOWN_ERROR;

    for (i = 0; i < 16; i++) {
      rclnt_rs.SonarController.SonarSet[0].Sonar[i].Reading =
	((double) SensorData[i].reading);

      if (rclnt_rs.SonarController.SonarSet[0].TimeStampActive)
	rclnt_rs.SonarController.SonarSet[0].Sonar[i].TimeStamp =
	  SensorData[i].sens_time;
    }
    break;
    
  case 'x': 
    break;
    
  }
  
  return 0;
}

int N_GetBumper(long int robot_id)
{
  int i; 
  struct I100_sensor_data SensorData; 

  if ((rclnt_connected == 0) || (robot_id != rclnt_robot_id))
    return N_ROBOT_NOT_FOUND;

  switch (rclnt_robot_type) {
  case 'n': 
    if (!rclnt_rs.BumperController.BumperSet[0].DataActive)
      return 0;

    /* WARNING lseek needed here?? */

    if (read(Bumper, &SensorData, sizeof(SensorData)) < 0)
      return N_UNKNOWN_ERROR;

    for (i = 0; i < 20; i++) {
      rclnt_rs.BumperController.BumperSet[0].Bumper[i].Reading =
	SensorData.reading % 2; 
      SensorData.reading /= 2;
      

      if (rclnt_rs.BumperController.BumperSet[0].TimeStampActive)
	rclnt_rs.BumperController.BumperSet[0].Bumper[i].TimeStamp =
	  SensorData.sens_time;
    }
    
    break;
    
  case 'x': 
    break;
  }

  return 0;
}

int N_SetTimer(long int robot_id)
{
  if ((rclnt_connected == 0) || (robot_id != rclnt_robot_id))
    return N_ROBOT_NOT_FOUND;

  RobotTimeout = rclnt_rs.Timer.Timeout;
  
  return 0;
}

int N_SetAxes(long int robot_id)
{
  if ((rclnt_connected == 0) || (robot_id != rclnt_robot_id))
    return N_ROBOT_NOT_FOUND;

  return I200M_SetAxes(&rclnt_rs);
}

int N_GetAxes(long int robot_id)
{
  if ((rclnt_connected == 0) || (robot_id != rclnt_robot_id))
    return N_ROBOT_NOT_FOUND;

  return I200M_GetAxes(&rclnt_rs);
  
}

int N_GetIntegratedConfiguration(long int robot_id)
{
  if ((rclnt_connected == 0) || (robot_id != rclnt_robot_id))
    return N_ROBOT_NOT_FOUND;

  return I200M_GetIntegratedConfiguration(&rclnt_rs);
}

int N_SetIntegratedConfiguration(long int robot_id)
{
  if ((rclnt_connected == 0) || (robot_id != rclnt_robot_id))
    return N_ROBOT_NOT_FOUND;

  return I200M_SetIntegratedConfiguration(&rclnt_rs);
}

int N_GetTimer(long int robot_id)
{
  if ((rclnt_connected == 0) || (robot_id != rclnt_robot_id))
    return N_ROBOT_NOT_FOUND;

  rclnt_rs.Timer.Timeout = RobotTimeout;
  rclnt_rs.Timer.Time = I200M_MSecSinceBoot();
  return 0;
}

int N_GetBattery(long int robot_id)
{
  int i; 
  unsigned char data[4]; 

  if ((rclnt_connected == 0) || (robot_id != rclnt_robot_id))
    return N_ROBOT_NOT_FOUND;

  if (!rclnt_rs.BatterySet.DataActive)
    return 0;

  switch (rclnt_battery_type) {
  case RCLNT_I100_BATTERY: 
    if (read(rclnt_battery_fd, &data, 2) < 0)
      return N_UNKNOWN_ERROR;
    for (i = 0; i < (int)rclnt_rs.BatterySet.BatteryCount; i++) {
      rclnt_rs.BatterySet.Battery[i].Voltage = data[i];
    }
    break;
    
  case RCLNT_PDB_BATTERY: 
    break;

  default: 
    assert(0);
  } 
  
  return 0;
}

int N_GetState(long int robot_id)
{
  if ((rclnt_connected == 0) || (robot_id != rclnt_robot_id))
    return N_ROBOT_NOT_FOUND;

  N_GetAxes(robot_id);
  N_GetIntegratedConfiguration(robot_id);
  N_GetSonar(robot_id);
  N_GetBumper(robot_id);
  N_GetInfrared(robot_id);
  N_GetBattery(robot_id);
  N_GetTimer(robot_id);

  return 0;
}

int N_Speak(long int robot_id, const char *text)
{
  if ((rclnt_connected == 0) || (robot_id != rclnt_robot_id))
    return N_ROBOT_NOT_FOUND;

  if (Sound < 0)
    return N_DEVICE_NOT_FOUND;
  
  if (write(Sound, text, strlen(text) + 1) < 0)
    return N_UNKNOWN_ERROR;
  
  return 0;
}

void N_ResetMotionTimer()
{
  I200M_ResetMotionTimer();
  return;
}
