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
#include "navigator_ackerman_messages.h"
#include "navigator_ackerman_interface.h"

static void
handler(carmen_navigator_ackerman_autonomous_stopped_message *msg 
	__attribute__ ((unused)))
{
  carmen_die("Received autonomous stopped\n");
}

int main(int argc, char **argv)
{
  carmen_robot_and_trailers_traj_point_t goal;
  carmen_navigator_ackerman_status_message *status;

  carmen_ipc_initialize(argc, argv);

  carmen_navigator_ackerman_query_status(&status);
  carmen_warn("status %f %f %f\n", status->robot.x,
	      status->robot.y, carmen_radians_to_degrees(status->robot.theta));
  goal.x = status->robot.x;
  goal.y = status->robot.y;
  goal.theta = carmen_normalize_theta(status->robot.theta + M_PI);
  free(status);
  carmen_navigator_ackerman_set_goal_triplet(&goal);
 
  carmen_navigator_ackerman_subscribe_autonomous_stopped_message
    (NULL, (carmen_handler_t)handler, CARMEN_SUBSCRIBE_LATEST);
 
  carmen_navigator_ackerman_go();
  carmen_ipc_dispatch();
  
  return 0;
}

