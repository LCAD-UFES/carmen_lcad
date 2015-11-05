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

#include "global.h"
#include "multicentral.h"

void test_subscribe_messages(void)
{

}

void test_ipc_exit_handler(void)
{
  fprintf(stderr, "Central died.\n");
}

int main(int argc, char **argv)
{
  carmen_centrallist_p centrallist;

  /* set this if it is OK for the program to run without connections
     to any centrals */
  carmen_multicentral_allow_zero_centrals(1);

  /* connect to all IPC servers */
  centrallist = carmen_multicentral_initialize(argc, argv, 
					       test_ipc_exit_handler);

  /* start thread that monitors connections to centrals */
  carmen_multicentral_start_central_check(centrallist);

  /* subscribe to messages from each central */
  carmen_multicentral_subscribe_messages(centrallist, 
					 test_subscribe_messages);

  do {
    /* handle IPC messages across all centrals */
    carmen_multicentral_ipc_sleep(centrallist, 0.1);

    /* attempt to reconnect any missing centrals */
    carmen_multicentral_reconnect_centrals(centrallist, NULL,
					   test_subscribe_messages);
  } while(1);
  return 0;
}
