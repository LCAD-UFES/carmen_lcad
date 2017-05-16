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
#include <carmen/global.h>
#include <carmen/param_interface.h>
#include <carmen/multicentral.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/bumblebee_basic_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/traffic_light_interface.h>
#include <carmen/traffic_light_messages.h>

IPC_CONTEXT_PTR car02_context;
IPC_CONTEXT_PTR car01_context;
IPC_CONTEXT_PTR current_context;
int camera_id = 4;

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_rddf_annotation_copy(carmen_rddf_annotation_message *rddf_annotation_message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_RDDF_ANNOTATION_MESSAGE_NAME, rddf_annotation_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_RDDF_ANNOTATION_MESSAGE_NAME);
}

///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
carmen_traffic_light_message_handler(carmen_traffic_light_message *message)
{
	if (IPC_getContext() != car02_context)
		return;

	IPC_setContext(car01_context);
	carmen_traffic_light_publish_message(camera_id, message);
	IPC_setContext(car02_context);
}


void
carmen_localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	if (IPC_getContext() != car01_context)
		return;

	IPC_setContext(car02_context);
	carmen_localize_ackerman_publish_globalpos_message(msg);
	IPC_setContext(car01_context);
}


void
carmen_rddf_annotation_message_handler(carmen_rddf_annotation_message *msg)
{
	if (IPC_getContext() != car01_context)
		return;

	IPC_setContext(car02_context);
	publish_rddf_annotation_copy(msg);
	IPC_setContext(car01_context);
}


////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
register_message()
{
	carmen_rddf_define_messages();
	carmen_localize_ackerman_define_globalpos_messages();
}


void multicentral_subscribe_messages(void)
{
	carmen_localize_ackerman_subscribe_globalpos_message(NULL,
			(carmen_handler_t) carmen_localize_ackerman_globalpos_message_handler,CARMEN_SUBSCRIBE_LATEST);
	carmen_rddf_subscribe_annotation_message(NULL,
			(carmen_handler_t) carmen_rddf_annotation_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_traffic_light_subscribe(camera_id, NULL,
			(carmen_handler_t) carmen_traffic_light_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


//////////////////////////////////////////////////////////////////////////////////////////////////



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
	centrallist = carmen_multicentral_initialize(argc, argv, test_ipc_exit_handler);

	/* start thread that monitors connections to centrals */
	carmen_multicentral_start_central_check(centrallist);

	/* subscribe to messages from each central */
	carmen_multicentral_subscribe_messages(centrallist, multicentral_subscribe_messages);

	if (centrallist->num_centrals > 0)
	{
		if (strcmp(centrallist->central[0].host, "alberto-Inspiron-7559") == 0)
		{
			car02_context = centrallist->central[0].context;
			car01_context = centrallist->central[1].context;
		}
		else
		{
			car02_context = centrallist->central[1].context;
			car01_context = centrallist->central[0].context;
		}

		current_context = IPC_getContext();
		IPC_setContext(car02_context);
		register_message();
		IPC_setContext(car01_context);
		carmen_traffic_light_define_messages(camera_id);
		IPC_setContext(current_context);
	}

	do {
		/* handle IPC messages across all centrals */
		carmen_multicentral_ipc_sleep(centrallist, 0.1);

		/* attempt to reconnect any missing centrals */
		carmen_multicentral_reconnect_centrals(centrallist, NULL,
				multicentral_subscribe_messages);
	} while(1);
	return 0;
}
