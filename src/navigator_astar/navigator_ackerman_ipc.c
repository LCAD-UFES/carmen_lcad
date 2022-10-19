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

#ifndef NO_ZLIB
#include <zlib.h>
#endif

#include "planner_ackerman_interface.h"
#include "navigator_ackerman_ipc.h"
#include "navigator_astar.hpp"


void carmen_navigator_ackerman_publish_plan_tree(carmen_robot_and_trailers_traj_point_t *p1, carmen_robot_and_trailers_traj_point_t *p2, int num_edges)
{
	static int first_time = 1;

	IPC_RETURN_TYPE err;
	carmen_navigator_ackerman_plan_tree_message msg;

	if(first_time) {

		err = IPC_defineMsg(
				CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME,
				IPC_VARIABLE_LENGTH,
				CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_FMT);

		carmen_test_ipc_exit(err,
				"Could not define message",
				CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME);
		first_time = 0;
	}

	if (p1 == NULL)
		return;

	if (p2 == NULL)
		return;
	//msg.p1 = (carmen_point_p)calloc(num_edges,sizeof(carmen_point_t));
	//msg.p2 = (carmen_point_p)calloc(num_edges,sizeof(carmen_point_t));

	msg.p1 = p1;
	msg.p2 = p2;
	msg.num_edges = num_edges;

	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();
	err = IPC_publishData(CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME, &msg);
	carmen_test_ipc_exit(err, "Could not publish",
			CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME);
}

int carmen_navigator_ackerman_initialize_ipc(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_NAVIGATOR_ACKERMAN_MAP_REQUEST_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_NAVIGATOR_ACKERMAN_MAP_REQUEST_FMT);
	carmen_test_ipc_exit(err, "Could not define message",
			CARMEN_NAVIGATOR_ACKERMAN_MAP_REQUEST_NAME);

	err = IPC_defineMsg(CARMEN_NAVIGATOR_ACKERMAN_MAP_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_NAVIGATOR_ACKERMAN_MAP_FMT);
	carmen_test_ipc_exit(err, "Could not define message",
			CARMEN_NAVIGATOR_ACKERMAN_MAP_NAME);

	return 0;
}
