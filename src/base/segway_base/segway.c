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
#include "segwaycore.h"
#include "segway_ipc.h"

int quit_signal = 0;
segway_t segway;
double accel_factor, torque_factor;
int gain_schedule;
double command_tv = 0, command_rv = 0;
double last_command = 0, last_status = 0;

void shutdown_handler(int sig)
{
  if(sig == SIGINT) { 
    fprintf(stderr, "\n");
    quit_signal = 1;
  }
}

void read_parameters(int argc, char **argv)
{
  char *schedule;

  carmen_param_t param[] = {
    {"segway", "accel_limit", CARMEN_PARAM_DOUBLE, &accel_factor, 0, NULL},
    {"segway", "torque_limit", CARMEN_PARAM_DOUBLE, &torque_factor, 0, NULL},
    {"segway", "gain_schedule", CARMEN_PARAM_STRING, &schedule, 0, NULL},
  };
  carmen_param_install_params(argc, argv, param, sizeof(param) / 
			      sizeof(param[0]));
  if(strncmp("heavy", schedule, 5) == 0)
    gain_schedule = 2;
  else if(strncmp("tall", schedule, 4) == 0)
    gain_schedule = 1;
  else 
    gain_schedule = 0;
}

int main(int argc, char **argv)
{
  double current_time;
  double last_battery = 0;

  /* connect to IPC network */
  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);
  carmen_segway_register_messages();
  
  read_parameters(argc, argv);

  segway_initialize(&segway);
  segway_clear_status(&segway);
  signal(SIGINT, shutdown_handler);
  siginterrupt(SIGINT, 1);

  segway_set_max_velocity(&segway, 1.0);
  segway_set_max_acceleration(&segway, accel_factor);
  segway_set_max_torque(&segway, torque_factor);
  segway_set_gain_schedule(&segway, SEGWAY_LIGHT);

  do {
    if(segway.status_ready) {
      current_time = carmen_get_time();
      if(current_time - last_command > 1.0) {
	command_tv = 0.0;
	command_rv = 0.0;
	last_command = current_time;
      }

      segway_set_velocity(&segway, command_tv, command_rv);
      current_time = carmen_get_time();
      carmen_segway_publish_pose(&segway, current_time);
      carmen_segway_publish_odometry(&segway, current_time);
      carmen_ipc_sleep(0.001);

      if(current_time - last_status > 1.0) {
	fprintf(stderr, "\rTV = %.1f     RV = %.1f   Battery = %d%%      ",
		command_tv, command_rv, (int)segway.voltage);
	last_status = current_time;
      }

      if(current_time - last_battery > 10.0) {
	carmen_segway_publish_battery(&segway);
	last_battery = current_time;
      }

      segway_clear_status(&segway);
    }
    segway_update_status(&segway);
  } while(!quit_signal);
  fprintf(stderr, "\n");
  segway_stop(&segway);
  segway_free(&segway);
  return 0;
}

