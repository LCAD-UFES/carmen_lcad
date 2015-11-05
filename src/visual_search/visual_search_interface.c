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
#include <carmen/visual_search_messages.h>
#include <carmen/visual_search_interface.h>

//subscribes to the trainning message
void
carmen_visual_search_subscribe_train(carmen_visual_search_message *visual_search_message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_VISUAL_SEARCH_TRAINING_MESSAGE_NAME,
                           CARMEN_VISUAL_SEARCH_TRAINING_MESSAGE_FMT,
                           visual_search_message, sizeof(carmen_visual_search_message),
			   handler, subscribe_how);
  printf("\nSubscribe to Visual Search ! - Trainning Mode\n");
}

//unsubscribe to the trainning message
void
carmen_visual_search_unsubscribe_train(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_VISUAL_SEARCH_TRAINING_MESSAGE_NAME, handler);
}

//subscribes to the test message
void
carmen_visual_search_subscribe_test(carmen_visual_search_test_message *visual_search_message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_VISUAL_SEARCH_TEST_MESSAGE_NAME,
                           CARMEN_VISUAL_SEARCH_TEST_MESSAGE_FMT,
                           visual_search_message, sizeof(carmen_visual_search_test_message),
			   handler, subscribe_how);
  printf("\nSubscribe to Visual Search ! - Testing Mode\n");
}

//unsubscribe to the test message
void
carmen_visual_search_unsubscribe_test(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_VISUAL_SEARCH_TEST_MESSAGE_NAME, handler);
}

//subscribes to the output message
void
carmen_visual_search_subscribe_output(carmen_visual_search_output_message *visual_search_message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_VISUAL_SEARCH_OUTPUT_MESSAGE_NAME,
                           CARMEN_VISUAL_SEARCH_OUTPUT_MESSAGE_FMT,
                           visual_search_message, sizeof(carmen_visual_search_output_message),
			   handler, subscribe_how);
  printf("\nSubscribe to Visual Search ! - Output Mode\n");
}

//unsubscribe to the output message
void
carmen_visual_search_unsubscribe_output(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_VISUAL_SEARCH_OUTPUT_MESSAGE_NAME, handler);
}

//subscribes to the state change message
void
carmen_visual_search_subscribe_state_change(carmen_visual_search_state_change_message *visual_search_state_change_message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_VISUAL_SEARCH_STATE_CHANGE_MESSAGE_NAME,
                           CARMEN_VISUAL_SEARCH_STATE_CHANGE_MESSAGE_FMT,
                           visual_search_state_change_message, sizeof(carmen_visual_search_state_change_message),
			   handler, subscribe_how);
  printf("\nSubscribe to Visual Search ! - State Changes\n");
}

//unsubscribes to the state change message
void
carmen_visual_search_unsubscribe_state_change(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_VISUAL_SEARCH_STATE_CHANGE_MESSAGE_NAME, handler);
}

IPC_RETURN_TYPE
carmen_visual_search_define_message_train(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_VISUAL_SEARCH_TRAINING_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VISUAL_SEARCH_TRAINING_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VISUAL_SEARCH_TRAINING_MESSAGE_NAME);
	return(err);
}

IPC_RETURN_TYPE
carmen_visual_search_define_message_test(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_VISUAL_SEARCH_TEST_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VISUAL_SEARCH_TEST_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VISUAL_SEARCH_TEST_MESSAGE_NAME);
	return(err);
}

IPC_RETURN_TYPE
carmen_visual_search_define_message_output(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_VISUAL_SEARCH_OUTPUT_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VISUAL_SEARCH_OUTPUT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VISUAL_SEARCH_OUTPUT_MESSAGE_NAME);
	return(err);
}

IPC_RETURN_TYPE
carmen_visual_search_define_message_state_change(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_VISUAL_SEARCH_STATE_CHANGE_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VISUAL_SEARCH_STATE_CHANGE_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VISUAL_SEARCH_STATE_CHANGE_MESSAGE_NAME);
	return(err);
}

IPC_RETURN_TYPE
carmen_visual_search_define_message_query(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_VISUAL_SEARCH_QUERY_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VISUAL_SEARCH_QUERY_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VISUAL_SEARCH_STATE_CHANGE_MESSAGE_NAME);
	return(err);
}

void
carmen_visual_search_subscribe_query(HANDLER_TYPE handler)
{
	IPC_RETURN_TYPE err;
	err = IPC_subscribe(CARMEN_VISUAL_SEARCH_QUERY_MESSAGE_NAME, handler, NULL);
	carmen_test_ipc(err, "Could not subscribe", CARMEN_VISUAL_SEARCH_QUERY_MESSAGE_NAME);
	IPC_setMsgQueueLength(CARMEN_VISUAL_SEARCH_QUERY_MESSAGE_NAME, 1);
}

carmen_visual_search_output_message *
carmen_visual_search_query_output_message(carmen_visual_search_test_message *query, double timeout)
{
	IPC_RETURN_TYPE err;
	carmen_visual_search_output_message *response = 0;
	unsigned int timeoutMsecs = (unsigned int) timeout * 1000.0;

	err = IPC_queryResponseData(CARMEN_VISUAL_SEARCH_QUERY_MESSAGE_NAME, query, (void **)&response, timeoutMsecs);

	if (err != IPC_OK)
	{
		carmen_test_ipc(err, "Could not get visual search output", CARMEN_VISUAL_SEARCH_QUERY_MESSAGE_NAME);
		return NULL;
	}

	return(response);

}
