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
#include <carmen/visual_tracker_messages.h>
#include <carmen/visual_tracker_interface.h>

//subscribes to the training message
void
carmen_visual_tracker_subscribe_train(carmen_visual_tracker_train_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message((char *) CARMEN_VISUAL_TRACKER_TRAIN_MESSAGE_NAME,
		  (char *) CARMEN_VISUAL_TRACKER_TRAIN_MESSAGE_FMT,
          message, sizeof(carmen_visual_tracker_train_message),
		  handler, subscribe_how);
  printf("\nSubscribe to Visual tracker ! - Trainning Mode\n");
}

//unsubscribe to the training message
void
carmen_visual_tracker_unsubscribe_train(carmen_handler_t handler)
{
  carmen_unsubscribe_message((char *) CARMEN_VISUAL_TRACKER_TRAIN_MESSAGE_NAME, handler);
}

//subscribes to the test message
void
carmen_visual_tracker_subscribe_test(carmen_visual_tracker_test_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message((char *) CARMEN_VISUAL_TRACKER_TEST_MESSAGE_NAME,
		  (char *) CARMEN_VISUAL_TRACKER_TEST_MESSAGE_FMT,
          message, sizeof(carmen_visual_tracker_test_message),
		  handler, subscribe_how);
  printf("\nSubscribe to Visual tracker ! - Testing Mode\n");
}

//unsubscribe to the test message
void
carmen_visual_tracker_unsubscribe_test(carmen_handler_t handler)
{
  carmen_unsubscribe_message((char *) CARMEN_VISUAL_TRACKER_TEST_MESSAGE_NAME, handler);
}

//subscribes to the output message
void
carmen_visual_tracker_subscribe_output(carmen_visual_tracker_output_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message((char *) CARMEN_VISUAL_TRACKER_OUTPUT_MESSAGE_NAME,
		  (char *) CARMEN_VISUAL_TRACKER_OUTPUT_MESSAGE_FMT,
          message, sizeof(carmen_visual_tracker_output_message),
		  handler, subscribe_how);
  printf("\nSubscribe to Visual tracker ! - output Mode\n");
}

//unsubscribe to the output message
void
carmen_visual_tracker_unsubscribe_output(carmen_handler_t handler)
{
  carmen_unsubscribe_message((char *) CARMEN_VISUAL_TRACKER_OUTPUT_MESSAGE_NAME, handler);
}

IPC_RETURN_TYPE
carmen_visual_tracker_define_message_train(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg((char *) CARMEN_VISUAL_TRACKER_TRAIN_MESSAGE_NAME, IPC_VARIABLE_LENGTH, (char *) CARMEN_VISUAL_TRACKER_TRAIN_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", (char *) CARMEN_VISUAL_TRACKER_TRAIN_MESSAGE_NAME);
	return(err);
}

IPC_RETURN_TYPE
carmen_visual_tracker_define_message_test(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg((char *) CARMEN_VISUAL_TRACKER_TEST_MESSAGE_NAME, IPC_VARIABLE_LENGTH, (char *) CARMEN_VISUAL_TRACKER_TEST_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", (char *) CARMEN_VISUAL_TRACKER_TEST_MESSAGE_NAME);
	return(err);
}

IPC_RETURN_TYPE
carmen_visual_tracker_define_message_output(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg((char *) CARMEN_VISUAL_TRACKER_OUTPUT_MESSAGE_NAME, IPC_VARIABLE_LENGTH, (char *) CARMEN_VISUAL_TRACKER_OUTPUT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", (char *) CARMEN_VISUAL_TRACKER_OUTPUT_MESSAGE_NAME);
	return(err);
}
