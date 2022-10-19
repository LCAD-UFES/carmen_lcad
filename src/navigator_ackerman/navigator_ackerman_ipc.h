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

#ifndef NAVIGATOR_ACKERMAN_IPC_H
#define NAVIGATOR_ACKERMAN_IPC_H

#include "navigator_ackerman_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

  int carmen_navigator_ackerman_initialize_ipc(void);
  void carmen_navigator_ackerman_publish_status(void);
  void carmen_navigator_ackerman_publish_plan(void);
  void carmen_navigator_ackerman_publish_autonomous_stopped(carmen_navigator_ackerman_reason_t reason);
  void carmen_navigator_ackerman_publish_plan_tree(carmen_robot_and_trailers_traj_point_t *p1, carmen_robot_and_trailers_traj_point_t *p2, int num_edges);
  void carmen_navigator_initialize_ant(int port, char *module);


#ifdef __cplusplus
}
#endif

#endif
