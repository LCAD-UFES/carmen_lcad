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
#include "map_io.h"

/* signal handler for C^c */
void 
shutdown_mapServer(int x)
{
  if (x == SIGINT) {
    carmen_ipc_disconnect();
    exit(1);
  }  
}

static void 
read_parameters(int argc, char **argv)
{
  if (argc != 2)
    carmen_die("Usage: %s <map filename>\n", argv[0]);

  if (carmen_file_exists(argv[1]) == 0)
    carmen_die("Could not find file %s\n", argv[1]);

  carmen_map_set_filename(argv[1]);
}


/* main */
int 
main(int argc, char** argv)
{
  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);

  read_parameters(argc, argv);

  if(carmen_map_initialize_ipc() < 0) 
    carmen_die("\nError: Could not initialize IPC.\n");  

  signal(SIGINT, shutdown_mapServer);

  while(1) {
    carmen_ipc_sleep(1.0);
  }
   
  return 0;
}
