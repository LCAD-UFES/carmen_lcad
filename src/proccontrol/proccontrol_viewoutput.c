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
#include <carmen/param_interface.h>
#include <carmen/proccontrol_interface.h>

void output_handler(carmen_proccontrol_output_message *output)
{
  fprintf(stderr, "%s", output->output);
}

int main(int argc, char **argv)
{
  /* connect to the IPC server, regsiter messages */
  carmen_ipc_initialize(argc, argv);

  carmen_proccontrol_subscribe_output_message(NULL, (carmen_handler_t)
					      output_handler,
					      CARMEN_SUBSCRIBE_ALL);
  carmen_ipc_dispatch();
  return 0;
}
