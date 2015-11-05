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

#ifndef CARMEN_MULTICENTRAL_H
#define CARMEN_MULTICENTRAL_H

typedef struct {
  int connected, ready_for_reconnect;
  char host[256];
  IPC_CONTEXT_PTR context;
} carmen_central_t, *carmen_central_p;

typedef struct {
  int num_centrals;
  carmen_central_p central;
} carmen_centrallist_t, *carmen_centrallist_p;

#ifdef __cplusplus
extern "C" {
#endif

void carmen_multicentral_allow_zero_centrals(int allow);

carmen_centrallist_p carmen_multicentral_initialize(int argc, char **argv,
						    void (*exit_handler)(void));

void carmen_multicentral_get_params(carmen_centrallist_p centrallist, 
				    int argc, char **argv,
				    void (*param_func)(int, char **));

void carmen_multicentral_register_messages(carmen_centrallist_p centrallist,
					   void (*register_func)(void));

void carmen_multicentral_subscribe_messages(carmen_centrallist_p centrallist,
					    void (*subscribe_func)(void));

void carmen_multicentral_reconnect_centrals(carmen_centrallist_p centrallist,
					    void (*register_func)(void),
					    void (*subscribe_func)(void));

void carmen_multicentral_start_central_check(carmen_centrallist_p centrallist);

void carmen_multicentral_ipc_sleep(carmen_centrallist_p centrallist, 
				   double sleep_time);

#ifdef __cplusplus
}
#endif

#endif
