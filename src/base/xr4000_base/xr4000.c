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
#include "xr4000_control.h"

#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "Nclient.h"
#include "arcnet_sensors.h"
#include "default_setup.h"
#include "setup.h"
#include "robot_client.h"

char *robd_program_name; 
int robd_robot_id = 1;
static unsigned char robd_done;

static void SignalTrap(int signal_num)
{
  fprintf(stderr, "%s: exiting on signal %d.\n", robd_program_name, 
	  signal_num);
  robd_done = 1;
  return;
}

int main(int argc, char **argv)
{
  FILE *setup_stream; 
  double last_publish = 0;
  char template[13] = "xr4000XXXXXX";
  int fd;

  /* connect to IPC network */
  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);

  /* remove path from program name */
  if(strrchr(argv[0], '/') != NULL)
    robd_program_name = strrchr(argv[0], '/') + 1;
  else
    robd_program_name = argv[0];

  /* write setup string to a file, and then read it back in. 
     This is pretty screwed up way of doing things. MM */
  fd = mkstemp(template);
  if(fd < 0) {
    fprintf(stderr, "Error: Could not get temporary filename.\n");
    exit(-1);
  }
  setup_stream = fdopen(fd, "w");
  fputs(robd_default_setup_string, setup_stream);
  fclose(setup_stream);
  SETUP_ReadSetup(template);
  unlink(template);
  SETUP_ReadSetup("/usr/local/xrdev/etc/nrobot.cfg");
  
  /* Initialize hardware */
  N_InitializeClient(NULL, 0);
  N_ConnectRobot(robd_robot_id);
  INIT_InitializeSonars(robd_robot_id);
  printf("%s: initialized.\n", robd_program_name);

  /* run the main socket & scheduler loop */
  robd_done = 0;
  signal(SIGINT, SignalTrap);
  signal(SIGTERM, SignalTrap);
  signal(SIGHUP, SignalTrap);

  /* initialize communications stuff */
  carmen_xr4000_start();
  while (!robd_done) {
    carmen_ipc_sleep(0.01);
    N_PollClient();
    if(carmen_get_time() - last_publish > 0.1) {
      carmen_xr4000_run();
      N_ResetMotionTimer();
      last_publish = carmen_get_time();
    }
  } 

  /* disconnect from robot hardware */
  N_DisconnectRobot(robd_robot_id);
  return 0;
}
