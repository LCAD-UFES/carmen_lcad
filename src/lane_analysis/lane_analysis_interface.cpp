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
#include <carmen/lane_analysis_messages.h>
#include <carmen/lane_analysis_interface.h>
// #include "lane_analysis_messages.h" // TODO: remove


// ==============================================================
// UNSUBSCRIBES
void carmen_elas_lane_estimation_unsubscribe(carmen_handler_t handler) { carmen_unsubscribe_message((char *)CARMEN_ELAS_LANE_ESTIMATION_NAME, handler); }
void carmen_elas_lane_markings_type_unsubscribe(carmen_handler_t handler) { carmen_unsubscribe_message((char *)CARMEN_ELAS_LANE_MARKINGS_TYPE_NAME, handler); }
void carmen_elas_adjacent_lanes_unsubscribe(carmen_handler_t handler) { carmen_unsubscribe_message((char *)CARMEN_ELAS_ADJACENT_LANES_NAME, handler); }


// ==============================================================
// SUBSCRIBES
void
carmen_elas_lane_estimation_subscribe(carmen_elas_lane_estimation_message * message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char *)CARMEN_ELAS_LANE_ESTIMATION_NAME, (char *)CARMEN_ELAS_LANE_ESTIMATION_FMT,
			message, sizeof(carmen_elas_lane_estimation_message),
			handler, subscribe_how);
}
void
carmen_elas_lane_markings_type_subscribe(carmen_elas_lane_markings_type_message * message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char *)CARMEN_ELAS_LANE_MARKINGS_TYPE_NAME, (char *)CARMEN_ELAS_LANE_MARKINGS_TYPE_FMT,
			message, sizeof(carmen_elas_lane_markings_type_message),
			handler, subscribe_how);
}
void
carmen_elas_adjacent_lanes_subscribe(carmen_elas_adjacent_lanes_message * message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char *)CARMEN_ELAS_ADJACENT_LANES_NAME, (char *)CARMEN_ELAS_ADJACENT_LANES_FMT,
			message, sizeof(carmen_elas_adjacent_lanes_message),
			handler, subscribe_how);
}

// ==============================================================
// PUBLISHES
IPC_RETURN_TYPE
carmen_elas_lane_estimation_publish_message(carmen_elas_lane_estimation_message * message)
{
	IPC_RETURN_TYPE err;
    err = IPC_publishData(CARMEN_ELAS_LANE_ESTIMATION_NAME, message);
    carmen_test_ipc_exit(err, "Could not publish!", CARMEN_ELAS_LANE_ESTIMATION_NAME);

    return err;
}
IPC_RETURN_TYPE
carmen_elas_lane_markings_type_publish_message(carmen_elas_lane_markings_type_message * message)
{
	IPC_RETURN_TYPE err;
    err = IPC_publishData(CARMEN_ELAS_LANE_MARKINGS_TYPE_NAME, message);
    carmen_test_ipc_exit(err, "Could not publish!", CARMEN_ELAS_LANE_MARKINGS_TYPE_NAME);

    return err;
}
IPC_RETURN_TYPE
carmen_elas_adjacent_lanes_publish_message(carmen_elas_adjacent_lanes_message * message)
{
	IPC_RETURN_TYPE err;
    err = IPC_publishData(CARMEN_ELAS_ADJACENT_LANES_NAME, message);
    carmen_test_ipc_exit(err, "Could not publish!", CARMEN_ELAS_ADJACENT_LANES_NAME);

    return err;
}
