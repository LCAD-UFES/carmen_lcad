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

carmen_laser_laser_message frontLaserMessage, rearLaserMessage;

void frontLaserHandler(void)
{
  carmen_warn("F");
}

void rearLaserHandler(void)
{
  carmen_warn("R");
}

void sonarHandler(void)
{
  carmen_warn("S");
}

void shutdown_module(int x)
{
  if (x == SIGINT)
    {
      carmen_warn("\n");
      carmen_ipc_disconnect();
      printf("shut down\n");
      exit(0);
    }
}

int main(int argc, char **argv)
{
  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);

  signal(SIGINT, shutdown_module);
 
  carmen_laser_subscribe_frontlaser_message(&frontLaserMessage,
					    (carmen_handler_t)
					    frontLaserHandler,
					    CARMEN_SUBSCRIBE_LATEST);
  carmen_laser_subscribe_rearlaser_message(&rearLaserMessage,
					   (carmen_handler_t)
					   rearLaserHandler,
					   CARMEN_SUBSCRIBE_LATEST);

  while(1) {
    carmen_ipc_sleep(0.1);
    fprintf(stderr, ".");
  }
  return 0;
}
