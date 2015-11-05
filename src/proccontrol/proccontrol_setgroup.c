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
#include <carmen/proccontrol_interface.h>

int main(int argc, char **argv)
{
  int requested_state = 0;

  if(argc < 3)
    carmen_die("Error: not enough arguments.\n"
	    "Usage: %s groupname requested-state\n", argv[0]);
  
  if(strcmp(argv[2], "0") == 0 ||
     strcmp(argv[2], "OFF") == 0 ||
     strcmp(argv[2], "DOWN") == 0)
    requested_state = 0;
  else if(strcmp(argv[2], "1") == 0 ||
          strcmp(argv[2], "ON") == 0 ||
          strcmp(argv[2], "UP") == 0)
    requested_state = 1;
  else
    carmen_die("Error: requested state %s invalid.\n", argv[2]);

  /* connect to the IPC server */
  carmen_ipc_initialize(argc, argv);
  carmen_proccontrol_set_group_state(argv[1], requested_state);
  return 0;
}
