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
#include "simulator_ackerman_interface.h"

int main(int argc, char *argv[]) { 
  
  int i,j;
  IPC_RETURN_TYPE ret;

  if (argc < 3 || carmen_find_param("-help") || 
      carmen_find_param("--help") || carmen_find_param("-h")) {

    fprintf(stderr,"\n%s is used to connect serveral carmen-simulators\n", argv[0]);
    fprintf(stderr,"so that the individual robots can see each other.\n\n");
    fprintf(stderr,"Usage: %s [centralhost of 1st simulator] [...] [n-th simulator]\n", argv[0]);
    fprintf(stderr,"(e.g. %s localhost localhost:1382 locahost:1383)\n", argv[0]);
    exit(0);
  }

  for (i=1; i<argc; i++) {
    for (j=1; j<argc; j++) {
      if (i != j) {
	ret = IPC_connectModule(argv[0], argv[j]); 
	
	if (ret != IPC_OK) {
	  carmen_die("Error: could not connect to central on: %s\n", argv[j]);
	}
	
	carmen_simulator_ackerman_connect_robots(argv[i]); 
      }
    }
  }
  fprintf(stderr,"simulators are now successly connected!\n");
  return 0; 
}
