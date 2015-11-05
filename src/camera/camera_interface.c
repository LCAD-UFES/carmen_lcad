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
#include "camera_messages.h"
#include "camera_interface.h"

static carmen_camera_image_message *image_message_pointer_external = NULL;
static carmen_handler_t image_message_handler_external = NULL;

static void 
image_interface_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
			void *clientData __attribute__ ((unused)))
{
  IPC_RETURN_TYPE err = IPC_OK;
  FORMATTER_PTR formatter;
  
  formatter = IPC_msgInstanceFormatter(msgRef);

  if(image_message_pointer_external) {
    if(image_message_pointer_external->image != NULL)
      free(image_message_pointer_external->image);
      
    err = IPC_unmarshallData(formatter, callData, 
			     image_message_pointer_external,
			     sizeof(carmen_camera_image_message));
    }
  
  IPC_freeByteArray(callData);
  carmen_test_ipc_return(err, "Could not unmarshall", 
			 IPC_msgInstanceName(msgRef));
  if(image_message_handler_external)
    image_message_handler_external(image_message_pointer_external);
}

void
carmen_camera_subscribe_images(carmen_camera_image_message *image,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  IPC_RETURN_TYPE err = IPC_OK;
  
  err = IPC_defineMsg(CARMEN_CAMERA_IMAGE_NAME, IPC_VARIABLE_LENGTH, 
		      CARMEN_CAMERA_IMAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define message", 
		       CARMEN_CAMERA_IMAGE_NAME);

  if(subscribe_how == CARMEN_UNSUBSCRIBE) {
    IPC_unsubscribe(CARMEN_CAMERA_IMAGE_NAME, image_interface_handler);
    return;
  }

  if (image) {
    image_message_pointer_external = image;
    memset(image_message_pointer_external, 0, 
	   sizeof(carmen_camera_image_message));
  } else if (image_message_pointer_external == NULL) {
    image_message_pointer_external = (carmen_camera_image_message *)
      calloc(1, sizeof(carmen_camera_image_message));
    carmen_test_alloc(image_message_pointer_external);
  }
  
  image_message_handler_external = handler;
  err = IPC_subscribe(CARMEN_CAMERA_IMAGE_NAME, image_interface_handler, NULL);
  if (subscribe_how == CARMEN_SUBSCRIBE_LATEST)
    IPC_setMsgQueueLength(CARMEN_CAMERA_IMAGE_NAME, 1);
  else
    IPC_setMsgQueueLength(CARMEN_CAMERA_IMAGE_NAME, 100);
  carmen_test_ipc(err, "Could not subscribe", CARMEN_CAMERA_IMAGE_NAME);
}
