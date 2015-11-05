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

/* handler for C^c */
void 
shutdown_module(int x)
{
  if(x == SIGINT) {
    carmen_ipc_disconnect();
    exit(1);
  }
}

void
handler(char *module, char *variable, char *value)
{
  fprintf(stderr, "%s_%s has changed, is now %s\n", module, variable, value);
}

int 
main(int argc, char** argv)
{
  double length;
  char *dev;
  int num_items;
  char **modules;
  int num_modules;

  carmen_param_t param_list[] = {
    {"robot", "length", CARMEN_PARAM_DOUBLE, &length, 1, handler},
    {"scout", "dev", CARMEN_PARAM_STRING, &dev, 1, handler},
  };

  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);

  signal(SIGINT, shutdown_module);
  
  num_items = sizeof(param_list)/sizeof(param_list[0]);

  carmen_param_install_params(argc, argv, param_list, num_items);
 
  carmen_param_get_robot();
  carmen_param_get_modules(&modules, &num_modules);
 
  carmen_warn("robot_length is %.1f\n", length);
  carmen_warn("scout_dev is %s\n", dev);

  carmen_param_check_unhandled_commandline_args(argc, argv);

  carmen_ipc_dispatch();

  return 0;
}
