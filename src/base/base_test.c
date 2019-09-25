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
#include <carmen/base_interface.h>

carmen_base_odometry_message odometry;
carmen_base_sonar_message sonar;

void base_odometry_handler(void)
{
  fprintf(stderr, "O");
}

void base_sonar_handler(void)
{
  fprintf(stderr, "S");
}

void shutdown_module(int x)
{
  if(x == SIGINT) {
    carmen_ipc_disconnect();
    printf("Disconnected.\n");
    exit(1);
  }
}

static void 
base_velocity_command(double tv, double rv)
{
  IPC_RETURN_TYPE err;
  static carmen_base_velocity_message v;

  v.tv = tv;
  v.rv = rv;
  v.host = carmen_get_host();
  v.timestamp = carmen_get_time();

  err = IPC_publishData(CARMEN_BASE_VELOCITY_NAME, &v);
  carmen_test_ipc(err, "Could not publish", CARMEN_BASE_VELOCITY_NAME);  
}

int main(int argc, char **argv)
{
  int count = 0;

  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);

  signal(SIGINT, shutdown_module);

  carmen_base_subscribe_odometry_message(&odometry, (carmen_handler_t)
					 base_odometry_handler,
					 CARMEN_SUBSCRIBE_LATEST);
  carmen_base_subscribe_sonar_message(&sonar, (carmen_handler_t)
				      base_sonar_handler,
				      CARMEN_SUBSCRIBE_LATEST);

	while (1) {
		base_velocity_command(0.0, carmen_degrees_to_radians(40.0));
		sleep(1);
	}

  while(1) {
    carmen_ipc_sleep(4.0);

    fprintf(stderr, ".");
    /*    if(count % 2 == 0)
      base_velocity_command(0.0, carmen_degrees_to_radians(40.0));
    else if(count % 2)
    base_velocity_command(0.0, carmen_degrees_to_radians(-40.0));*/
    count++;
  }
  return 0;
}

