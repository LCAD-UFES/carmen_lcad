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
 * any later version. *
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

carmen_laser_laser_message frontlaser, rearlaser;

double first_timestamp;
int front_laser_count = 0, rear_laser_count = 0;

void laser_frontlaser_handler(void)
{
  front_laser_count++;
}

void laser_rearlaser_handler(void)
{
  rear_laser_count++;
}

void shutdown_module(int x __attribute__ ((unused)))
{
  carmen_ipc_disconnect();
  printf("\nDisconnected from laser.\n");
  exit(1);  
}

void print_statistics(void *clientdata __attribute__ ((unused)), 
		      unsigned long t1 __attribute__ ((unused)), 
		      unsigned long t2 __attribute__ ((unused)))
{
  double current_timestamp = carmen_get_time();

  fprintf(stderr, "L1 - %.2f Hz      L2 - %.2f Hz\n", 
	  front_laser_count / (current_timestamp - first_timestamp),
	  rear_laser_count / (current_timestamp - first_timestamp));
}

int main(int argc, char **argv)
{
  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);
  signal(SIGINT, shutdown_module);
 
  carmen_laser_subscribe_frontlaser_message(&frontlaser, (carmen_handler_t)
					    laser_frontlaser_handler, 
					    CARMEN_SUBSCRIBE_LATEST);
  carmen_laser_subscribe_rearlaser_message(&rearlaser, (carmen_handler_t)
					   laser_rearlaser_handler, 
					   CARMEN_SUBSCRIBE_LATEST);

  carmen_ipc_addPeriodicTimer(1000, print_statistics, NULL);

  first_timestamp = carmen_get_time();
  carmen_ipc_dispatch();
  return 0;
}
