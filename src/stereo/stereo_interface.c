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
#include <carmen/stereo_messages.h>
#include <carmen/stereo_interface.h>

char *
carmen_stereo_message_name(int camera)
{
  char *message_name = (char *) malloc(128 * sizeof(char));
  sprintf(message_name, "%s%d", (char *) CARMEN_SIMPLE_STEREO_DISPARITY_NAME, camera);
  return message_name;
}


void
carmen_stereo_unsubscribe(int camera, carmen_handler_t handler)
{
  char *message_name = carmen_stereo_message_name(camera);
  carmen_unsubscribe_message(message_name, handler);
  free(message_name);
}

IPC_RETURN_TYPE
carmen_stereo_define_messages(int camera)
{
  IPC_RETURN_TYPE err;

  char *message_name = carmen_stereo_message_name(camera);
  err = IPC_defineMsg(message_name, IPC_VARIABLE_LENGTH, (char *) CARMEN_SIMPLE_STEREO_DISPARITY_FMT);
  carmen_test_ipc_exit(err, "Could not define", message_name);
  free(message_name);
  return err;
}

void
carmen_stereo_subscribe(int camera,
    carmen_simple_stereo_disparity_message *message,
    carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
  char *message_name = carmen_stereo_message_name(camera);
  carmen_subscribe_message(message_name, (char *) CARMEN_SIMPLE_STEREO_DISPARITY_FMT,
      message, sizeof(carmen_simple_stereo_disparity_message),
      handler, subscribe_how);
  free(message_name);
  printf("\nSubscribed to Stereo Messages (From Camera %d)!\n", camera);
}

IPC_RETURN_TYPE
carmen_stereo_publish_message(int camera,
    carmen_simple_stereo_disparity_message *message)
{
  IPC_RETURN_TYPE err;

  char *message_name = carmen_stereo_message_name(camera);

  err = IPC_publishData(message_name, message);
  carmen_test_ipc_exit(err, (char *) "Could not publish", message_name);
  free(message_name);

  return err;
}

