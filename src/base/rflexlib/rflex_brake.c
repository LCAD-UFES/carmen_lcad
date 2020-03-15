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

#include <errno.h>
#include <carmen/carmen.h>

#include "rflex_brake.h"

void
usage( char *prgname )
{
  fprintf( stderr,
	   "Usage: %s <brake>\n  0: brake off, 1: brake on\n",
	   prgname );
}
     
int
main( int argc, char *argv[] )
{
  IPC_RETURN_TYPE err;
  carmen_rflex_brake_message data;

  if (argc!=2) {
    usage(argv[0]);
    exit(1);
  }

  carmen_ipc_initialize(argc, argv);
  
  data.set_brake  = atoi(argv[1]);

  err = IPC_publishData (CARMEN_RFLEX_BRAKE_NAME, &data );
  carmen_test_ipc(err, "Could not publish", CARMEN_RFLEX_BRAKE_NAME);

  exit(0);
}
