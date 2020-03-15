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
#include <carmen/bumblebee_basic_messages.h>

char *
carmen_bumblebee_basic_message_name(int camera)
{
	char *message_name = (char*)malloc(128 * sizeof(char));
	sprintf(message_name, "%s%d", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE_NAME, camera);
	return message_name;
}

void
carmen_bumblebee_basic_unsubscribe_stereoimage(int camera, carmen_handler_t handler)
{
	char *message_name = carmen_bumblebee_basic_message_name(camera);
	carmen_unsubscribe_message(message_name, handler);
	free(message_name);
}

IPC_RETURN_TYPE
carmen_bumblebee_basic_define_messages(int camera)
{
  IPC_RETURN_TYPE err;

  char *message_name = carmen_bumblebee_basic_message_name(camera);
  err = IPC_defineMsg(message_name, IPC_VARIABLE_LENGTH, CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define", message_name);
  free(message_name);
  return err;
}

void
carmen_bumblebee_basic_subscribe_stereoimage(int camera,
		carmen_bumblebee_basic_stereoimage_message *message,
		carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	char *message_name = carmen_bumblebee_basic_message_name(camera);
	carmen_subscribe_message(message_name, CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE_FMT,
			message, sizeof(carmen_bumblebee_basic_stereoimage_message),
			handler, subscribe_how);
	free(message_name);
	printf("\nSubscribe to Bumblebee(Camera %d) Images!\n", camera);
}

IPC_RETURN_TYPE
carmen_bumblebee_basic_publish_message(int camera,
		const carmen_bumblebee_basic_stereoimage_message *message)
{
	IPC_RETURN_TYPE err;

	char *message_name = carmen_bumblebee_basic_message_name(camera);

	err = IPC_publishData(message_name, (carmen_bumblebee_basic_stereoimage_message *)message);
	carmen_test_ipc_exit(err, "Could not publish", message_name);
	free(message_name);

	return err;
}
