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

#include <carmen/carmen.h>
#include "Nclient.h"
#include "xr4000_control.h"
#include "xr4000_ipc.h"
#include "xr4000.h"

typedef struct {
  double max_tv, max_rv;
  double t_accel, r_accel;
  struct N_RobotState *rs;
  struct N_Configuration *config;
} xr4000_t, *xr4000_p;

xr4000_t robot;
extern int robd_robot_id;

static void carmen_xr4000_publish_odometry(void)
{
  carmen_base_odometry_message odometry;
  double xv = robot.rs->AxisSet.Axis[N_XTRANSLATION].DesiredSpeed / 10.0;
  double yv = robot.rs->AxisSet.Axis[N_YTRANSLATION].DesiredSpeed / 10.0;
  double rv = robot.rs->AxisSet.Axis[N_ROTATION].DesiredSpeed / 1000.0;
  IPC_RETURN_TYPE err;

  /* get robot position */
  N_GetIntegratedConfiguration(1);
  odometry.x = robot.rs->Integrator.y / 1000.0;
  odometry.y = -robot.rs->Integrator.x / 1000.0;
  odometry.theta = 
    carmen_normalize_theta(robot.rs->Integrator.Rotation / 1000.0);
  odometry.tv = hypot(xv, yv);
  odometry.rv = rv;
  odometry.acceleration = robot.t_accel;
  odometry.timestamp = carmen_get_time();
  odometry.host = carmen_get_host();

  err = IPC_publishData(CARMEN_BASE_ODOMETRY_NAME, &odometry);
  carmen_test_ipc_exit(err, "Could not publish", CARMEN_BASE_ODOMETRY_NAME);
}

static void carmen_xr4000_execute_velocity(void)
{
  /* execute robot velocity */
  if(N_SetAxes(robd_robot_id) != N_NO_ERROR)
    carmen_warn("Could not set robot velocity.\n");
}

void carmen_xr4000_usage(char *progname, char *fmt, ...) 
{
  va_list args;

  if(fmt != NULL) {
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fprintf(stderr, "\n\n");
  }
  else
    fprintf(stderr, "\n");
  if(strrchr(progname, '/') != NULL) {
    progname = strrchr(progname, '/');
    progname++;
  }
  fprintf(stderr, "Usage: %s <args>\n", progname);
  fprintf(stderr, "\t-dev devname   - device connected to scout robot.\n");
  exit(-1);
}

void carmen_xr4000_set_velocity(double xv, double yv, double rv)
{
  /* set robot velocity */
  xv *= 1000;
  if(xv > XR4000_MAX_TV)
    xv = XR4000_MAX_TV;
  yv *= 1000;
  if(yv > XR4000_MAX_TV)
    yv = XR4000_MAX_TV;
  rv *= 1000;
  if(rv > XR4000_MAX_TV)
    rv = XR4000_MAX_TV;
  robot.rs->AxisSet.Axis[N_XTRANSLATION].DesiredSpeed = (long)(xv);
  robot.rs->AxisSet.Axis[N_XTRANSLATION].Update = TRUE;
  robot.rs->AxisSet.Axis[N_YTRANSLATION].DesiredSpeed = (long)(yv);
  robot.rs->AxisSet.Axis[N_YTRANSLATION].Update = TRUE;
  robot.rs->AxisSet.Axis[N_ROTATION].DesiredSpeed = (long)(rv);
  robot.rs->AxisSet.Axis[N_ROTATION].Update = TRUE;
}

void carmen_xr4000_start(void)
{
  /* get the robot state */
  robot.rs = N_GetRobotState(robd_robot_id);
  if(robot.rs == NULL)
    carmen_die("Error: could not get robot state.\n");

  /* set robot control parameters */
  robot.rs->AxisSet.Global = FALSE;
  robot.rs->AxisSet.Axis[N_XTRANSLATION].Mode = N_AXIS_VELOCITY;
  robot.rs->AxisSet.Axis[N_XTRANSLATION].Acceleration = XR4000_T_ACCEL;
  robot.rs->AxisSet.Axis[N_YTRANSLATION].Mode = N_AXIS_VELOCITY;
  robot.rs->AxisSet.Axis[N_YTRANSLATION].Acceleration = XR4000_T_ACCEL;
  robot.rs->AxisSet.Axis[N_ROTATION].Mode = N_AXIS_VELOCITY;
  robot.rs->AxisSet.Axis[N_ROTATION].Acceleration = XR4000_R_ACCEL;

  carmen_xr4000_set_velocity(0, 0, 0);
  carmen_xr4000_register_ipc_messages();
}

void carmen_xr4000_run(void)
{
  carmen_xr4000_execute_velocity();
  carmen_xr4000_publish_odometry();
}

static void carmen_xr4000_stop_robot(void)
{
  /* stop all robot motion */
  robot.rs->AxisSet.Axis[N_XTRANSLATION].Mode = N_AXIS_STOP;
  robot.rs->AxisSet.Axis[N_XTRANSLATION].Update = TRUE;
  robot.rs->AxisSet.Axis[N_YTRANSLATION].Mode = N_AXIS_STOP;
  robot.rs->AxisSet.Axis[N_YTRANSLATION].Update = TRUE;
  robot.rs->AxisSet.Axis[N_ROTATION].Mode = N_AXIS_STOP;
  robot.rs->AxisSet.Axis[N_ROTATION].Update = TRUE;
  if(N_SetAxes(robd_robot_id) != N_NO_ERROR)
    carmen_warn("Failure when stopping robot.\n");
}

void carmen_xr4000_shutdown(int sig)
{
  if(sig == SIGINT)
    carmen_xr4000_stop_robot();
}

void carmen_xr4000_emergency_crash(int sig __attribute ((unused)))
{
  carmen_xr4000_stop_robot();
}



